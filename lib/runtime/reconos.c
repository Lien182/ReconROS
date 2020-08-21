/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Runtime library
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Markus Happe, University of Paderborn
 *                 Sebastian Meisner, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  ReconOS runtime library managing all threads and
 *                 internal data structures. It provides functions
 *                 to manipulate the state of the system.
 *
 * ======================================================================
 */

#include "reconos.h"
#include "private.h"
#include "arch/arch.h"
#include "comp/mbox.h"
#include "comp/ros.h"
#include "comp/ros_pub.h"
#include "comp/ros_sub.h"
#include "comp/ros_service_server.h"
#include "comp/ros_action_server.h"
#include "comp/ros_service_client.h"
#include "comp/ros_action_client.h"

#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <sys/mman.h>
#include <limits.h>

int RECONOS_NUM_HWTS = 0;

static struct hwslot *_hwslots;
static int _proc_control;
static int _clock;
static pthread_t _pgf_handler;
static struct sigaction _dt_signal;
static int _thread_id;


/* == ReconOS resource ================================================= */

/*
 * @see header
 */
void reconos_resource_init(struct reconos_resource *rr,
                           int type, void *ptr) {
	rr->type = type;
	rr->ptr = ptr;
}


/* == ReconOS clock ==================================================== */

/*
 * @see header
 */
int reconos_clock_set(int clk, int m, int f)
{
	int divider = 100000 * m / f;
	reconos_clock_set_divider(_clock, clk, divider);

	return 100000 * m / divider;
}


/* == ReconOS thread =================================================== */

/*
 * @see header
 */
void reconos_thread_init(struct reconos_thread *rt,
                         char* name,
                         int state_size) {
	int i;

	rt->name = name;
	rt->id = _thread_id++;

	rt->init_data = NULL;
	rt->resources = NULL;
	rt->resource_count = 0;

	rt->state = RECONOS_THREAD_STATE_INIT;
	rt->state_data = NULL;

	if (state_size != 0) {
		rt->state_data = malloc(state_size);
		if (!rt->state_data) {
			panic("[reconos-core] ERROR: failed to allocate memory for state\n");
		}
		memset((void *)rt->state_data, 0, state_size);
	}

	rt->allowed_hwslot_count = RECONOS_NUM_HWTS;
	rt->allowed_hwslots = (struct hwslot **)malloc(RECONOS_NUM_HWTS * sizeof(struct hwslot *));
	if (!rt->allowed_hwslots) {
		panic("[reconos-core] ERROR: failed to allocate memory for allowed hwslots\n");
	}

	for (i = 0; i < rt->allowed_hwslot_count; i++) {
		rt->allowed_hwslots[i] = &_hwslots[i];
	}

	rt->hwslot = NULL;

	rt->bitstreams = NULL;
	rt->bitstream_lengths = NULL;
	rt->thread_priority = 0;
}

/*
 * @see header
 */
void reconos_thread_setinitdata(struct reconos_thread *rt, void *init_data) {
	rt->init_data = init_data;
}

/*
 * @see header
 */
void reconos_thread_setallowedslots(struct reconos_thread *rt,
                                    int *slots, int slot_count) {
	int i;

	if (rt->allowed_hwslots) {
		free(rt->allowed_hwslots);
	}

	rt->allowed_hwslot_count = slot_count;
	rt->allowed_hwslots = (struct hwslot **)malloc(slot_count * sizeof(struct hwslot *));
	if (!rt->allowed_hwslots) {
		panic("[reconos-core] ERROR: failed to allocate memory for allowed hwslots\n");
	}

	for (i = 0; i < rt->allowed_hwslot_count; i++) {
		rt->allowed_hwslots[i] = &_hwslots[slots[i]];
	}
}

/*
 * @see header
 */
void reconos_thread_setresources(struct reconos_thread *rt,
                                 struct reconos_resource *resources,
                                 int resource_count) {
	rt->resources = resources;
	rt->resource_count = resource_count;
}

/*
 * @see header
 */
void reconos_thread_setresourcepointers(struct reconos_thread *rt,
                                        struct reconos_resource **resources,
                                        int resource_count) {
	int i;

	rt->resources = (struct reconos_resource *)malloc(resource_count * sizeof(struct reconos_resource));
	if (!rt->resources) {
		panic("[reconos-core] ERROR: failed to allocate memory for resources\n");
	}

	for (i = 0; i < resource_count; i++) {
		rt->resources[i] = *resources[i];
	}
	rt->resource_count = resource_count;
}

/*
 * @see header
 */
void reconos_thread_setbitstream(struct reconos_thread *rt,
                                 char **bitstreams,
                                 int *bitstream_lengths) {
	rt->bitstreams = bitstreams;
	rt->bitstream_lengths = bitstream_lengths;
}

/*
 * @see header
 */
void reconos_thread_loadbitstream(struct reconos_thread *rt,
                                  char *path) {
	FILE *file;
	unsigned int i, size;

	debug("[reconos-core] loading bitstreams from %s\n", path);

	rt->bitstream_lengths = (int *)malloc(RECONOS_NUM_HWTS * sizeof(int));
	if (!rt->bitstream_lengths) {
		panic("[reconos-core] ERROR: failed to allocate memory for bitstream");
	}

	rt->bitstreams = (char **)malloc(RECONOS_NUM_HWTS * sizeof(char *));
	if (!rt->bitstreams) {
		panic("[reconos-core] ERROR: failed to allocate memory for bitstream");
	}

	for (i = 0; i < RECONOS_NUM_HWTS; i++) {
		file = fopen(path, "rb");
		if (!file) {
			panic("[reconos-core] ERROR: failed to open bitstream\n");
		}

		fseek(file, 0L, SEEK_END);
		size = ftell(file);
		rewind(file);

		rt->bitstream_lengths[i] = size;
		rt->bitstreams[i] = (char *)malloc(size * sizeof(char));
		if (!rt->bitstreams[i]) {
			panic("[reconos-core] ERROR: failed to allocate memory for bitstream\n");
		}

		fread(rt->bitstreams[i], sizeof(char), size, file);

		fclose(file);
	}
}

/*
 * @see header
 */
void reconos_thread_setswentry(struct reconos_thread *rt,
                               void *(*swentry)(void *data)) {
	rt->swentry = swentry;
}

/*
 * @see header
 */
void reconos_thread_create(struct reconos_thread *rt, int slot) {
	int i;

	if (slot < 0 || slot >= RECONOS_NUM_HWTS) {
		panic("[reconos-core] ERROR: slot id out of range\n");
	}

	if (rt->state == RECONOS_THREAD_STATE_RUNNING_HW) {
		panic("[reconos-core] ERROR: thread is already running\n");
	}

	for (i = 0; i < rt->allowed_hwslot_count; i++) {
		if (rt->allowed_hwslots[i]->id == _hwslots[slot].id) {
			break;
		}
	}
	if (i == rt->allowed_hwslot_count) {
		panic("[reconos-core] ERROR: thread not allowed to run in slot\n");
	}

	rt->hwslot = &_hwslots[slot];
	hwslot_createthread(rt->hwslot, rt);
	rt->state = RECONOS_THREAD_STATE_RUNNING_HW;
}

/*
 * @see header
 */
void reconos_thread_create_auto(struct reconos_thread *rt, int tt) {
	int i;

	if (tt & RECONOS_THREAD_HW) {
		for (i = 0; i < rt->allowed_hwslot_count; i++) {
			if (!rt->allowed_hwslots[i]->rt) {
				break;
			}
		}
		if (i == rt->allowed_hwslot_count) {
			whine("[reconos_core] WARNING: no free slot for thread found");
			return;
		}

		reconos_thread_create(rt, rt->allowed_hwslots[i]->id);
	} else if (tt & RECONOS_THREAD_SW) {
		if(rt->thread_priority != 0)
		{
			struct sched_param param;
			pthread_attr_t attr;
			int ret;

			/* Lock memory */
			if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
					printf("mlockall failed: %m\n");
					exit(-2);
			}

			/* Initialize pthread attributes (default values) */
			ret = pthread_attr_init(&attr);
			if (ret) {
					printf("init pthread attributes failed\n");
					return;
			}


			/* Set a specific stack size  */
		//	ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
		//	if (ret) {
		//		printf("pthread setstacksize failed\n");
			//	return;
			//}

			/* Set scheduler policy and priority of pthread */
			ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
			if (ret) {
					printf("pthread setschedpolicy failed\n");
					return;
			}
			param.sched_priority = rt->thread_priority;
			ret = pthread_attr_setschedparam(&attr, &param);
			if (ret) 
			{
				printf("pthread setschedparam failed\n");
				return;
			}
			/* Use scheduling parameters of attr */
			ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
			if (ret) {
				printf("pthread setinheritsched failed\n");
					return;
			}
			
			pthread_create(&rt->swslot, &attr, rt->swentry, (void*)rt);
		}
		else
		{
			pthread_create(&rt->swslot, NULL,  rt->swentry, (void*)rt);
		}
	}
}

/*
 * @see header
 */
void reconos_thread_suspend(struct reconos_thread *rt) {
	if (rt->state != RECONOS_THREAD_STATE_RUNNING_HW) {
		panic("[reconos-core] ERROR: cannot suspend not running thread\n");
	}

	rt->state = RECONOS_THREAD_STATE_SUSPENDING;
	hwslot_suspendthread(rt->hwslot);
	rt->state = RECONOS_THREAD_STATE_SUSPENDED;
}

/*
 * @see header
 */
void reconos_thread_suspend_block(struct reconos_thread *rt) {
	if (rt->state != RECONOS_THREAD_STATE_RUNNING_HW) {
		panic("[reconos-core] ERROR: cannot suspend not running thread\n");
	}

	rt->state = RECONOS_THREAD_STATE_SUSPENDING;
	hwslot_suspendthread(rt->hwslot);
	hwslot_jointhread(rt->hwslot);
	rt->state = RECONOS_THREAD_STATE_SUSPENDED;
}

/*
 * @see header
 */
void reconos_thread_resume(struct reconos_thread *rt, int slot) {
	int i;

	if (slot < 0 || slot >= RECONOS_NUM_HWTS) {
		panic("[reconos-core] ERROR: slot id out of range\n");
	}

	for (i = 0; i < rt->allowed_hwslot_count; i++) {
		if (rt->allowed_hwslots[i]->id == _hwslots[slot].id) {
			break;
		}
	}
	if (i == rt->allowed_hwslot_count) {
		panic("[reconos-core] ERROR: thread not allowed to run in slot\n");
	}

	hwslot_resumethread(rt->hwslot, rt);
	rt->state = RECONOS_THREAD_STATE_RUNNING_HW;
}

/*
 * @see header
 */
void reconos_thread_join(struct reconos_thread *rt) {
	hwslot_jointhread(rt->hwslot);
}

/*
 * @see header
 */
void reconos_thread_signal(struct reconos_thread *rt) {
	hwslot_setsignal(rt->hwslot, 1);
}

void reconos_thread_setpriority(struct reconos_thread * rt, int thread_priority)
{
	rt->thread_priority = thread_priority;
}


/* == General functions ================================================ */

/*
 * Signal handler for exiting the program
 *
 *   sig - signal
 */
void exit_signal(int sig) {
	reconos_cleanup();

	printf("[reconos-core] aborted\n");

	exit(0);
}

/*
 * Signal handler for ínterrupting a delegate thread
 *
 *   sig - signal
 */
void delegate_signal(int sig) {
	debug("[reconos-core] delegate received signal\n");
}

/*
 * @see header
 */
void reconos_init() {
	int i, osif;

	signal(SIGINT, exit_signal);
	signal(SIGTERM, exit_signal);
	signal(SIGABRT, exit_signal);

	_thread_id = 0;

	reconos_drv_init();

	_proc_control = reconos_proc_control_open();
	if (_proc_control < 0) {
		panic("[reconos-core] ERROR: unable to open proc control\n");
	}

	_clock = reconos_clock_open();
	if (_clock < 0) {
		panic("[reconos-core] ERROR: unable to open clock\n");
	}

	RECONOS_NUM_HWTS = reconos_proc_control_get_num_hwts(_proc_control);

	_hwslots = (struct hwslot *)malloc(RECONOS_NUM_HWTS * sizeof(struct hwslot));
	if (!_hwslots) {
		panic("[reconos-core] ERROR: unable to allocate memory for slots\n");
	}

	for (i = 0; i < RECONOS_NUM_HWTS; i++) {
		osif = reconos_osif_open(i);
		if (osif < 0)
			panic("[reconos-core] ERROR: unable to open osif %d\n", i);

		hwslot_init(&_hwslots[i], i, osif);
	}

	_dt_signal.sa_handler = delegate_signal;
	sigaction(SIGUSR1, &_dt_signal, NULL);

	reconos_proc_control_sys_reset(_proc_control);

	reconos_proc_control_set_pgd(_proc_control);

#ifdef RECONOS_OS_linux
	pthread_create(&_pgf_handler, NULL, proc_pgfhandler, NULL);
#endif
}

/*
 * @see header
 */
void reconos_cleanup() {
	reconos_proc_control_sys_reset(_proc_control);
}

/*
 * @see header
 */
void reconos_cache_flush() {
	reconos_proc_control_cache_flush(_proc_control);
}


/* == ReconOS proc control============================================== */

/*
 * @see header
 */
void *proc_pgfhandler(void *arg) {
	while (1) {
		uint32_t *addr;

		addr = (uint32_t *)reconos_proc_control_get_fault_addr(_proc_control);

		printf("[reconos_core] "
		       "page fault occured at address %x\n", (unsigned int)addr);

		*addr = 0;

		reconos_proc_control_clear_page_fault(_proc_control);
	}
}

/* == ReconOS hwslot =================================================== */

/*
 * @see header
 */
void hwslot_init(struct hwslot *slot, int id, int osif) {
	slot->id = id;
	slot->osif = osif;

	slot->rt = NULL;

	slot->dt = 0;
	slot->dt_state = DELEGATE_STATE_STOPPED;
	slot->dt_flags = 0;
	sem_init(&slot->dt_exit, 0, 0);
}

/*
 * @see header
 */
void hwslot_reset(struct hwslot *slot) {
	debug("[reconos-core] resetting slot %d\n", slot->id);
	reconos_proc_control_hwt_reset(_proc_control, slot->id, 1);
	reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
	reconos_proc_control_hwt_reset(_proc_control, slot->id, 0);
}

/*
 * @see header
 */
void hwslot_setreset(struct hwslot *slot, int reset) {
	reconos_proc_control_hwt_reset(_proc_control, slot->id, reset);
}

/*
 * @see header
 */
void hwslot_setsignal(struct hwslot *slot, int sig) {
	reconos_proc_control_hwt_signal(_proc_control, slot->id, sig);
}

/*
 * @see header
 */
void hwslot_createdelegate(struct hwslot *slot) {
	if (slot->dt) {
		panic("[reconos-core] ERROR: delegate thread already running\n");
	}

	
	slot->dt_state = DELEGATE_STATE_INIT;
	slot->dt_flags = 0;

	struct sched_param param;
	pthread_attr_t attr;
	int ret;

	/* Lock memory */
	//if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
	//		printf("mlockall failed: %m\n");
	//		exit(-2);
	//}

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);
	if (ret) {
			printf("init pthread attributes failed\n");
			return;
	}

	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (ret) {
			printf("pthread setschedpolicy failed\n");
			return;
	}
	param.sched_priority = 80;
	ret = pthread_attr_setschedparam(&attr, &param);
	if (ret) {
			printf("pthread setschedparam failed\n");
			return;
	}
	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
			printf("pthread setinheritsched failed\n");
			return;
	}




	pthread_create(&slot->dt, NULL, dt_delegate, slot);
}	

/*
 * @see header
 */
void hwslot_createthread(struct hwslot *slot,
                         struct reconos_thread *rt) {
	if (slot->rt) {
		panic("[reconos-core] ERROR: a thread is already running\n");
	}

	slot->rt = rt;

	reconos_proc_control_hwt_reset(_proc_control, slot->id, 1);
	reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
	reconos_proc_control_hwt_reset(_proc_control, slot->id, 0);

	reconos_osif_write(slot->osif, (uint32_t)OSIF_SIGNAL_THREAD_START);

	rt->thread_priority = 80;
	hwslot_createdelegate(slot);
}

/*
 * @see header
 */
void hwslot_suspendthread(struct hwslot *slot) {
	if (!slot->rt) {
		panic("[reconos-core] ERROR: no thread running\n");
	}

	slot->dt_flags |= DELEGATE_FLAG_PAUSE_SYSCALLS;
	slot->dt_flags |= DELEGATE_FLAG_SUSPEND;

	do {
		switch (slot->dt_state) {
			case DELEGATE_STATE_BLOCKED_OSIF:
				reconos_osif_break(slot->osif);
				break;

			case DELEGATE_STATE_BLOCKED_SYSCALL:
				pthread_kill(slot->dt, SIGUSR1);
				break;
		}

		sched_yield();
	} while (slot->dt_flags & DELEGATE_FLAG_SUSPEND);
}

/*
 * @see header
 */
void hwslot_resumethread(struct hwslot *slot,
                         struct reconos_thread *rt) {
	if (slot->rt) {
		panic("[reconos-core] ERROR: a thread is already running\n");
	}

	reconos_proc_control_hwt_reset(_proc_control, slot->id, 1);
	reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
	reconos_proc_control_hwt_reset(_proc_control, slot->id, 0);
	
	slot->rt = rt;
	
	slot->dt_flags &= ~DELEGATE_FLAG_PAUSE_SYSCALLS;
	slot->dt_flags &= ~DELEGATE_FLAG_SUSPEND;
	reconos_osif_write(slot->osif, (uint32_t)OSIF_SIGNAL_THREAD_RESUME);

}

/*
 * @see header
 */
void hwslot_jointhread(struct hwslot *slot) {
	if (!slot->rt) {
		panic("[reconos-core] ERROR: no thread running\n");
	}

	sem_wait(&slot->dt_exit);

	reconos_proc_control_hwt_reset(_proc_control, slot->id, 1);
	reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
	slot->rt = NULL;
}


/* == ReconOS delegate ================================================= */


#define RESOURCE_CHECK_TYPE(p_handle, p_type) \
	if ((p_handle) >= slot->rt->resource_count) {\
		panic("[reconos-dt-%d] "\
		      "ERROR: resource count out of range\n",slot->id);\
	}\
	if (slot->rt->resources[(p_handle)].type != (p_type)) {\
		panic("[reconos-dt-%d] "\
		      "ERROR: wrong resource type\n", slot->id);\
	}

#define SYSCALL_NONBLOCK(p_call)\
	if (slot->dt_flags & DELEGATE_FLAG_PAUSE_SYSCALLS) {\
		debug("[reconos-dt-%d] "\
		      "interrupted in nonblocking syscall\n", slot->id);\
		goto intr;\
	}\
	p_call;

#define SYSCALL_BLOCK(p_call)\
	if (slot->dt_flags & DELEGATE_FLAG_PAUSE_SYSCALLS) {\
		debug("[reconos-dt-%d] "\
		      "interrupted before blocking syscall\n", slot->id);\
		goto intr;\
	}\
	slot->dt_state = DELEGATE_STATE_BLOCKED_SYSCALL;\
	if ((p_call) < 0) {\
		debug("[reconos-dt-%d] "\
		      "interrupted in blocking syscall\n", slot->id);\
		goto intr;\
	}\
	slot->dt_state = DELEGATE_STATE_PROCESSING;

/*
 * Delegate function: Get initialization data
 *   Command: OSIF_CMD_GET_INIT_DATA
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_get_init_data(struct hwslot *slot) {
	reconos_osif_write(slot->osif, (uint32_t)slot->rt->init_data);

	return 0;
}

/*
 * Delegate function: Post Semaphore
 *   Command: OSIF_CMD_SEM_POST
 *   Syscall: sem_post
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_sem_post(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_SEM);

	debug("[reconos-dt-%d] (sem_post on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = sem_post(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (sem_post on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Acquire Semaphore
 *   Command: OSIF_CMD_SEM_WAIT
 *   Syscall: sem_wait
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_sem_wait(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_SEM);

	debug("[reconos-dt-%d] (sem_wait on %d) ...\n", slot->id, handle);
	SYSCALL_BLOCK(ret = sem_wait(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (sem_wait on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Lock a mutex
 *   Command: OSIF_CMD_MUTEX_LOCK
 *   Syscall: pthread_mutex_lock
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mutex_lock(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MUTEX);

	debug("[reconos-dt-%d] (mutex_lock on %d) ...\n", slot->id, handle);
	SYSCALL_BLOCK(ret = pthread_mutex_lock(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (mutex_lock on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Unlock a mutex
 *   Command: OSIF_CMD_MUTEX_UNLOCK
 *   Syscall: pthread_mutex_unlock
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mutex_unlock(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MUTEX);

	debug("[reconos-dt-%d] (mutex_unlock on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = pthread_mutex_unlock(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (mutex_unlock on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Tries to lock a mutex
 *   Command: OSIF_CMD_MUTEX_TRYLOCK
 *   Syscall: pthread_mutex_trylock
 *
 *   slot - pointer to the hardware slot
 */
static inline uint32_t dt_mutex_trylock(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MUTEX);

	debug("[reconos-dt-%d] (mutex_trylock on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = pthread_mutex_trylock(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (mutex_trylock on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Waits for a condition variable
 *   Command: OSIF_CMD_COND_WAIT
 *   Syscall: pthread_cond_wait
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_cond_wait(struct hwslot *slot) {
#ifndef RECONOS_MINIMAL
	int handle, handle2, ret;

	handle = reconos_osif_read(slot->osif);
	handle2 = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_COND);
	RESOURCE_CHECK_TYPE(handle2, RECONOS_RESOURCE_TYPE_MUTEX);

	debug("[reconos-dt-%d] (cond_wait on %d) ...\n", slot->id, handle);
	SYSCALL_BLOCK(ret = pthread_cond_wait(slot->rt->resources[handle].ptr,
	                                      slot->rt->resources[handle2].ptr));
	debug("[reconos-dt-%d] (cond_wait on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;
#else
	panic("[reconos-dt-%d] ERROR: (cond_wait on %d) not supported\n", slot->id, handle);

	return 0;
#endif

intr:
	return -1;
}

/*
 * Delegate function: Signals a condition variable
 *   Command: OSIF_CMD_COND_SIGNAL
 *   Syscall: pthread_cond_signal
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_cond_signal(struct hwslot *slot) {
#ifndef RECONOS_MINIMAL
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_COND);

	debug("[reconos-dt-%d] (cond_signal on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = pthread_cond_signal(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (cond_signal on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;
#else
	panic("[reconos-dt-%d] ERROR: (cond_signal on %d) not supported\n", slot->id, handle);

	return 0;
#endif

intr:
	return -1;
}

/*
 * Delegate function: Broadcasts a condition variable
 *   Command: OSIF_CMD_COND_BROADCAST
 *   Syscall: pthread_cond_broadcast
 *
 *   slot - pointer to the hardware slot
 */
static inline uint32_t dt_cond_broadcast(struct hwslot *slot) {
#ifndef RECONOS_MINIMAL
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_COND);

	debug("[reconos-dt-%d] (cond_broadcast on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = pthread_cond_broadcast(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (cond_broadcast on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;
#else
	panic("[reconos-dt-%d] ERROR: (cond_broadcast on %d) not supported\n", slot->id, handle);

	return 0;
#endif

intr:
	return -1;
}

/*
 * Delegate function: Gets a message from mbox
 *   Command: OSIF_CMD_MBOX_GET
 *   Syscall: mbox_get
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mbox_get(struct hwslot *slot) {
	int handle, ret;
	uint32_t msg;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MBOX);

	debug("[reconos-dt-%d] (mbox_get on %d) ...\n", slot->id, handle);
	SYSCALL_BLOCK(ret = mbox_get_interruptible(slot->rt->resources[handle].ptr, &msg));
	debug("[reconos-dt-%d] (mbox_get on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, msg);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Puts a message into mbox
 *   Command: OSIF_CMD_MBOX_PUT
 *   Syscall: mbox_put
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mbox_put(struct hwslot *slot) {
	int handle, ret;
	uint32_t arg0;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MBOX);

	arg0 = reconos_osif_read(slot->osif);

	debug("[reconos-dt-%d] (mbox_put on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = mbox_put(slot->rt->resources[handle].ptr, arg0));
	debug("[reconos-dt-%d] (mbox_put on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Tries to get a message from mbox
 *   Command: OSIF_CMD_MBOX_TRYGET
 *   Syscall: mbox_tryget
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mbox_tryget(struct hwslot *slot) {
	int handle, ret;
	uint32_t data;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MBOX);

	debug("[reconos-dt-%d] (mbox_tryget on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = mbox_tryget(slot->rt->resources[handle].ptr, &data));
	reconos_osif_write(slot->osif, data);
	debug("[reconos-dt-%d] (mbox_tryget on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

/*
 * Delegate function: Tries to put a message into mbox
 *   Command: OSIF_CMD_MBOX_TYPUT
 *   Syscall: mbox_tryput
 *
 *   slot - pointer to the hardware slot
 */
static inline int dt_mbox_tryput(struct hwslot *slot) {
	int handle, ret;
	uint32_t arg0;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_MBOX);

	arg0 = reconos_osif_read(slot->osif);

	debug("[reconos-dt-%d] (mbox_tryput on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = mbox_tryput(slot->rt->resources[handle].ptr, arg0));
	debug("[reconos-dt-%d] (mbox_tryput on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

static inline int dt_ros_publish(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSPUB);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSMSG);

	debug("[reconos-dt-%d] (ros publish on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_publisher_publish(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros publish on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

static inline int dt_ros_take(struct hwslot *slot) {
	int handle;
	int msg_handle;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSUB);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSMSG);

	debug("[reconos-dt-%d] (ros_take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ros_subscriber_message_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr););
	debug("[reconos-dt-%d] (ros_take on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_trytake(struct hwslot *slot) {
	int handle, ret, msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSUB);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSMSG);

	debug("[reconos-dt-%d] (ros_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_subscriber_message_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros_trytake on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_services_response(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGRES);

	debug("[reconos-dt-%d] (ros service response on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_service_server_response_send(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros service response on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

static inline int dt_ros_services_take(struct hwslot *slot) {
	int handle;
	int msg_handle;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGREQ);

	debug("[reconos-dt-%d] (ros service take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ros_service_server_request_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr););
	debug("[reconos-dt-%d] (ros service take %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_services_trytake(struct hwslot *slot) {
	int handle, ret, msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGREQ);

	debug("[reconos-dt-%d] (ros_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_service_server_request_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros_trytake on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

//ROS ACTION

static inline int dt_ros_actions_goal_take(struct hwslot *slot) {
	int handle;
	int msg_handle;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGGOALREQ);

	debug("[reconos-dt-%d] (ros service take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ros_action_server_goal_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr););
	debug("[reconos-dt-%d] (ros service take %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actions_goal_trytake(struct hwslot *slot) {
	int handle, ret, msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGGOALREQ);

	debug("[reconos-dt-%d] (ros_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_goal_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros_trytake on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actions_goal_decide(struct hwslot *slot) {
	int handle, ret;
	unsigned int accept;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);
	accept = reconos_osif_read(slot->osif);
	debug("[reconos-dt-%d] (ros_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_goal_decide(slot->rt->resources[handle].ptr, accept));
	debug("[reconos-dt-%d] (ros_trytake on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}


static inline int dt_ros_actions_result_take(struct hwslot *slot) {
	int handle, ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);

	debug("[reconos-dt-%d] (ros service take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_result_take(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (ros service take %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actions_result_trytake(struct hwslot *slot) {
	int handle, ret;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);


	debug("[reconos-dt-%d] (ros_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_result_try_take(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (ros_trytake on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}


static inline int dt_ros_actions_result_send(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGRESULTRES);

	debug("[reconos-dt-%d] (ros service response on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_result_send(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros service response on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}


static inline int dt_ros_actions_feedback(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONS);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGFEEDBACK);

	debug("[reconos-dt-%d] (ros service response on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_server_feedback(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros service response on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}


static inline int dt_ros_servicec_request(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGREQ);

	debug("[reconos-dt-%d] (ros service request on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_service_client_request_send(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros service request on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

static inline int dt_ros_servicec_take(struct hwslot *slot) {
	int handle;
	int msg_handle;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGRES);

	debug("[reconos-dt-%d] (ros service take response on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ros_service_client_response_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr););
	debug("[reconos-dt-%d] (ros service take response %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_servicec_trytake(struct hwslot *slot) {
	int handle, ret, msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSSRVC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSSRVMSGRES);

	debug("[reconos-dt-%d] (ros service try take response on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_service_client_response_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros service try take response on %d) done\n", slot->id, handle);

	
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}



static inline int dt_ros_actionc_goal_send(struct hwslot *slot) {
	int handle, msg_handle;
	int ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGGOALREQ);

	debug("[reconos-dt-%d] (ros action goal send on %d, msghandle %d) ...\n", slot->id, handle,msg_handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_goal_send(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr););
	debug("[reconos-dt-%d] (ros action goal send %d, msghandle %d) done\n", slot->id, handle,msg_handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}


static inline int dt_ros_actionc_goal_take(struct hwslot *slot) {
	int handle, ret;
	unsigned int accept = 0;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	
	debug("[reconos-dt-%d] (dt_ros_actionc_goal_take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_goal_take(slot->rt->resources[handle].ptr, &accept));
	debug("[reconos-dt-%d] (dt_ros_actionc_goal_take on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, accept);
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actionc_goal_trytake(struct hwslot *slot) {
	int handle, ret;
	unsigned int accept = 0;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	
	debug("[reconos-dt-%d] (dt_ros_actionc_goal_trytake on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_goal_try_take(slot->rt->resources[handle].ptr, &accept));
	debug("[reconos-dt-%d] (dt_ros_actionc_goal_trytake on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, accept);
	reconos_osif_write(slot->osif, (uint32_t)ret);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actionc_result_take(struct hwslot *slot) {
	int handle, ret, msg_handle;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGRESULTRES);

	
	
	debug("[reconos-dt-%d] (dt_ros_actionc_result_take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_result_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (dt_ros_actionc_result_take on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actionc_result_trytake(struct hwslot *slot) {
	int handle, msg_handle,ret;

	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGRESULTRES);

	debug("[reconos-dt-%d] (ros_action_client_result_try_take on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_result_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros_action_client_result_try_take on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)slot->rt->resources[msg_handle].ptr);
	

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actionc_result_request(struct hwslot *slot) {
	int handle, ret;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);

	debug("[reconos-dt-%d] (ros action result request on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_result_request(slot->rt->resources[handle].ptr));
	debug("[reconos-dt-%d] (ros action result request on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}


static inline int dt_ros_actionc_feedback_take(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGFEEDBACK);

	debug("[reconos-dt-%d] (ros action take feedback on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_feedback_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros action take feedback on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}

static inline int dt_ros_actionc_feedback_trytake(struct hwslot *slot) {
	int handle, ret;
	int msg_handle;
	handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(handle, RECONOS_RESOURCE_TYPE_ROSACTIONC);
	msg_handle = reconos_osif_read(slot->osif);
	RESOURCE_CHECK_TYPE(msg_handle, RECONOS_RESOURCE_TYPE_ROSACTIONMSGFEEDBACK);

	debug("[reconos-dt-%d] (ros action take feedback on %d) ...\n", slot->id, handle);
	SYSCALL_NONBLOCK(ret = ros_action_client_feedback_try_take(slot->rt->resources[handle].ptr, slot->rt->resources[msg_handle].ptr));
	debug("[reconos-dt-%d] (ros action take feedback on %d) done\n", slot->id, handle);

	reconos_osif_write(slot->osif, (uint32_t)ret);

	return 0;

intr:
	return -1;
}


typedef struct __dummy_dyn_ros_msg
{
  void * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} __dummy_dyn_ros_msg;


static inline int dt_ros_message_set_message_size(struct hwslot *slot) {
	unsigned int length = 0, offset = 0;
	int msg_handle;

	msg_handle = reconos_osif_read(slot->osif);

	offset = reconos_osif_read(slot->osif);
	length 	= reconos_osif_read(slot->osif);

	debug("[reconos-dt-%d] (ros_message_set_message_size on %d) ...\n", slot->id, msg_handle);
	__dummy_dyn_ros_msg * ros_msg = slot->rt->resources[msg_handle].ptr + offset;

	if(length != 0)
	{
		ros_msg->data = malloc(4*length); 
		ros_msg->capacity = length;
		ros_msg->size = length;
	}
	else
	{
		if(ros_msg->data != NULL)
			free(ros_msg->data); 
		ros_msg->capacity = 0;
		ros_msg->size = 0;
	}
	


	debug("[reconos-dt-%d] (ros_message_set_message_size on %d) done\n", slot->id, msg_handle);

	reconos_osif_write(slot->osif, (uint32_t)ros_msg->data);

	return 0;
}


static inline int dt_memory_malloc(struct hwslot *slot) {
	volatile uint32_t dummy = 0;
	unsigned int length = 0;

	void ** ptr_dest = (void **)reconos_osif_read(slot->osif);
	length 	= reconos_osif_read(slot->osif);

	debug("[reconos-dt-%d] (memory malloc with length=%d) ...\n", slot->id, length);
	*ptr_dest = malloc(length);
	((uint32_t*)(*ptr_dest))[0] = dummy;
	debug("[reconos-dt-%d] (memory malloc with length=%d) done\n", slot->id, length);

	reconos_osif_write(slot->osif, (uint32_t)*ptr_dest);

	return 0;
}

static inline int dt_memory_free(struct hwslot *slot) {
	void * ptr = (void *)reconos_osif_read(slot->osif);
	
	debug("[reconos-dt-%d] (memory free on %x) ...\n", slot->id, ptr);
	free(ptr);
	debug("[reconos-dt-%d] (memory free on %x) done\n", slot->id, ptr);
	return 0;
}

/*
 * @see header
 */
void *dt_delegate(void *arg) {
	struct hwslot *slot;
	uint32_t cmd;

	slot = (struct hwslot *)arg;

	slot->dt_state = DELEGATE_STATE_PROCESSING;

	while (1) {
		if (slot->dt_flags & DELEGATE_FLAG_SUSPEND) {
			slot->dt_flags &= ~DELEGATE_FLAG_SUSPEND;
			reconos_proc_control_hwt_signal(_proc_control, slot->id, 1);
		}

		debug("[reconos-dt-%d] waiting for command ...\n", slot->id);
		slot->dt_state = DELEGATE_STATE_BLOCKED_OSIF;
		cmd = reconos_osif_read(slot->osif);
		slot->dt_state = DELEGATE_STATE_PROCESSING;
		debug("[reconos-dt-%d] received command 0x%x\n", slot->id, cmd);

		switch (cmd & OSIF_CMD_MASK) {
			case OSIF_CMD_MBOX_PUT:
				dt_mbox_put(slot);
				break;

			case OSIF_CMD_MBOX_TRYPUT:
				dt_mbox_tryput(slot);
				break;

			case OSIF_CMD_SEM_POST:
				dt_sem_post(slot);
				break;

			case OSIF_CMD_MUTEX_UNLOCK:
				dt_mutex_unlock(slot);
				break;

			case OSIF_CMD_COND_SIGNAL:
				dt_cond_signal(slot);
				break;

			case OSIF_CMD_COND_BROADCAST:
				dt_cond_broadcast(slot);
				break;

			case OSIF_CMD_MBOX_GET:
				dt_mbox_get(slot);
				break;

			case OSIF_CMD_MBOX_TRYGET:
				dt_mbox_tryget(slot);
				break;

			case OSIF_CMD_SEM_WAIT:
				dt_sem_wait(slot);
				break;

			case OSIF_CMD_MUTEX_LOCK:
				dt_mutex_lock(slot);
				break;

			case OSIF_CMD_MUTEX_TRYLOCK:
				dt_mutex_trylock(slot);
				break;

			case OSIF_CMD_COND_WAIT:
				dt_cond_wait(slot);
				break;

			case OSIF_CMD_MEMORY_MALLOC:
				dt_memory_malloc(slot);
				break;
			
			case OSIF_CMD_MEMORY_FREE:
				dt_memory_free(slot);
				break;

			case OSIF_CMD_ROS_MESSAGE_SET_SIZE:
				dt_ros_message_set_message_size(slot);
				break;

			case OSIF_CMD_ROS_PUBLISH:
				dt_ros_publish(slot);
				break;

			case OSIF_CMD_ROS_TAKE:
				dt_ros_take(slot);
				break;

			case OSIF_CMD_ROS_TRYTAKE:
				dt_ros_trytake(slot);
				break;

			case OSIF_CMD_ROS_SERVICES_RESPONSE:
				dt_ros_services_response(slot);
				break;

			case OSIF_CMD_ROS_SERVICES_TAKE:
				dt_ros_services_take(slot);
				break;

			case OSIF_CMD_ROS_SERVICES_TRYTAKE:
				dt_ros_services_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_GOAL_TAKE:
				dt_ros_actions_goal_take(slot);
				break;
			
			case OSIF_CMD_ROS_ACTIONS_GOAL_TRYTAKE:
				dt_ros_actions_goal_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_GOAL_DECIDE:
				dt_ros_actions_goal_decide(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_RESULT_TAKE:
				dt_ros_actions_result_take(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_RESULT_TRYTAKE:
				dt_ros_actions_result_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_RESULT_SEND:
				dt_ros_actions_result_send(slot);
				break;

			case OSIF_CMD_ROS_ACTIONS_FEEDBACK:
				dt_ros_actions_feedback(slot);
				break;
			
			//ROS Service Client

			case OSIF_CMD_ROS_SERVICEC_REQUEST:
				dt_ros_servicec_request(slot);
				break;

			case OSIF_CMD_ROS_SERVICEC_TAKE:
				dt_ros_servicec_take(slot);
				break;

			case OSIF_CMD_ROS_SERVICEC_TRYTAKE:
				dt_ros_servicec_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_GOAL_SEND:
				dt_ros_actionc_goal_send(slot);
				break;
			
			case OSIF_CMD_ROS_ACTIONC_GOAL_TAKE:
				dt_ros_actionc_goal_take(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_GOAL_TRYTAKE:
				dt_ros_actionc_goal_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_RESULT_SEND:
				dt_ros_actionc_result_request(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_RESULT_TAKE:
				dt_ros_actionc_result_take(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_RESULT_TRYTAKE:
				dt_ros_actionc_result_trytake(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_FEEDBACK_TAKE:
				dt_ros_actionc_feedback_take(slot);
				break;

			case OSIF_CMD_ROS_ACTIONC_FEEDBACK_TRYTAKE:
				dt_ros_actionc_feedback_trytake(slot);
				break;	

			case OSIF_CMD_THREAD_GET_INIT_DATA:
				dt_get_init_data(slot);
				break;

			case OSIF_CMD_THREAD_EXIT:
				reconos_proc_control_hwt_reset(_proc_control, slot->id, 1);
				reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
				slot->dt_flags = 0;
				sem_post(&slot->dt_exit);
				break;

			case OSIF_CMD_THREAD_GET_STATE_ADDR:
				slot->dt_flags &= ~DELEGATE_FLAG_PAUSE_SYSCALLS;
				reconos_osif_write(slot->osif, (uint32_t)slot->rt->state_data);
				break;

			case OSIF_CMD_THREAD_CLEAR_SIGNAL:
				reconos_proc_control_hwt_signal(_proc_control, slot->id, 0);
				break;

			case OSIF_INTERRUPTED:
				break;

			default:
				panic("[reconos-dt-%d] ERROR received unknown command 0x%08x\n", slot->id, cmd);
				break;
		}

		debug("[reconos-dt-%d] executed command 0x%x\n", slot->id, cmd);
	}

return NULL;
}
