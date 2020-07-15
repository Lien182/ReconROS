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
 *                 Christoph Rüthing, University of Paderborn
 *   description:  ReconOS runtime library managing all threads and
 *                 internal data structures. It provides functions
 *                 to manipulate the state of the system.
 *
 * ======================================================================
 */

#ifndef RECONOS_H
#define RECONOS_H

#include <pthread.h>

#define RECONOS_VERSION_STRING "v3.1"

extern int RECONOS_NUM_HWTS;

/* == ReconOS resource ================================================= */

/*
 * Definition of the different resource types.
 *
 *   mbox  - mailbox (struct mbox)
 *   sem   - semaphore (sem_t)
 *   mutex - mutex (pthread_mutex)
 *   cond  - condition variable (pthread_cond)
 */
#define RECONOS_RESOURCE_TYPE_MBOX     		0x00000001
#define RECONOS_RESOURCE_TYPE_SEM     		0x00000002
#define RECONOS_RESOURCE_TYPE_MUTEX    		0x00000004
#define RECONOS_RESOURCE_TYPE_COND     		0x00000008
#define RECONOS_RESOURCE_TYPE_ROSNODE  		0x00000010	
#define RECONOS_RESOURCE_TYPE_ROSSUB 		0x00000020
#define RECONOS_RESOURCE_TYPE_ROSPUB		0x00000040
#define RECONOS_RESOURCE_TYPE_ROSMSG		0x00000080
#define RECONOS_RESOURCE_TYPE_ROSSRVS		0x00000100
#define RECONOS_RESOURCE_TYPE_ROSSRVC		0x00000200
#define RECONOS_RESOURCE_TYPE_ROSSRVMSGREQ	0x00000400
#define RECONOS_RESOURCE_TYPE_ROSSRVMSGRES	0x00000800
/*
 * Object representing a single resource.
 *
 *   type - type of the resource (RECONOS_RESOURCE_TYPE_...)
 *   ptr  - pointer to the representation of the resource
 */
struct reconos_resource {
	int type;
	void *ptr;
};

/*
 * Initializes the resource.
 * Simply assigning type and ptr is also appropriate.
 *
 *   rr   - pointer to the resource
 *   type - type of the resource
 *   ptr  - pointer to the representation of the resource
 */
void reconos_resource_init(struct reconos_resource *rr,
                           int type, void *ptr);


/* == ReconOS clock ==================================================== */

/*
 * Sets the divider for the given clock.
 *
 *   clk - the id of the clock
 *   m   - the internal multiplicator of the clock
 *   f   - the wanted frequency in kHz
 */
int reconos_clock_set(int clk, int m, int f);


/* == ReconOS thread =================================================== */

/*
 * Definition of thread types
 *
 *   sw - software thread
 *   hw - hardware thread
 */
#define RECONOS_THREAD_SW 0x01
#define RECONOS_THREAD_HW 0x02

/*
 * Definition of the thread states
 *
 *   stoped     - not created yet
 *   running_hw - executing as a hardware thread
 *   running_sw - executing as a software thread
 *   suspended  - suspendend and ready for sheduling
 *   suspending - currently suspending and saving state
 */
#define RECONOS_THREAD_STATE_INIT         0x01
#define RECONOS_THREAD_STATE_STOPED       0x02
#define RECONOS_THREAD_STATE_RUNNING_HW   0x04
#define RECONOS_THREAD_STATE_RUNNING_SW   0x08
#define RECONOS_THREAD_STATE_SUSPENDED    0x10
#define RECONOS_THREAD_STATE_SUSPENDING   0x20


/*
 * Object representing a hardware thread
 *
 *   init_data         - pointer to the initialization data
 *   resources         - array of resources associated
 *   resource_count    - number of resources in resource array
 *
 *   state             - current state (refers to RECONOS_THREAD_STATE_...)
 *   state_data        - memory to store the internal state
 *
 *   allowed_slots     - allowed slots to execute the thread in
 *   hw_slot           - hardware slot the thread is executing in
 *
 *   bitstreams        - array of bitstreams for the different slots
 *   bitstream_lengths - length of the bitstreams
 */
struct reconos_thread {
	char *name;
	int id;

	void *init_data;
	struct reconos_resource *resources;
	int resource_count;

	int state;
	volatile void *state_data;

	struct hwslot **allowed_hwslots;
	int allowed_hwslot_count;
	struct hwslot *hwslot;
	pthread_t swslot;
	int thread_priority;

	char **bitstreams;
	int *bitstream_lengths;
	void *(*swentry)(void *data);
};

/*
 * Initializes the ReconOS thread. Must be called before all other
 * methods.
 *
 *   rt         - pointer to the ReconOS thread
 *   name       - name of the thread (can be null)
 *   state_size - size in bytes for the state of the thread
 */
void reconos_thread_init(struct reconos_thread *rt,
                         char* name,
                         int state_size);

/*
 * Associates initialization data to this thread.
 *
 *   rt        - pointer to the ReconOS thread
 *   init_data - pointer to the initialization data
 */
void reconos_thread_setinitdata(struct reconos_thread *rt, void *init_data);

/*
 * Specifies the allowed threads the hardware thread is allowed to
 * run in.
 *
 *   rt         - pointer to the ReconOS thread
 *   slots      - array of allowed slot ids
 *   slot_count - number of slot ids in slot array
 */
void reconos_thread_setallowedslots(struct reconos_thread *rt,
                                    int *slots, int slot_count);

/*
 * Associates the resource array to this thread. The resource array
 * is used directly and no copy is created. Make shure to not modify
 * it and not put it on the stack.
 *
 *   rt             - pointer to the ReconOS thread
 *   resources      - array of resources
 *   resource_count - number of resources in resource array
 */
void reconos_thread_setresources(struct reconos_thread *rt,
                                 struct reconos_resource *resources,
                                 int resource_count);

/*
 * Associates the resource array to this thread. Therefore it allocates
 * memory and copies the provided resource structures. You can savely
 * free all memory allocated to pass data to this function.
 *
 *   rt             - pointer to the ReconOS thread
 *   resources      - array of pointers to resources
 *   resource_count - number of resources in resource array
 */

void reconos_thread_setresourcepointers(struct reconos_thread *rt,
                                        struct reconos_resource **resources,
                                        int resource_count);

/*
 * Assigns the bitstream array to the hardware thread. The bitstream
 * array must contain a bitstream for each hardware slot.
 *
 *   rt  - pointer to the ReconOS thread
 *   bitstreams - array of bitstreams (array of chars)
 *   bitstream_lengths - lengths of the different bitstreams
 */
void reconos_thread_setbitstream(struct reconos_thread *rt,
                                 char **bitstreams,
                                 int *bitstream_lengths);

/*
 * Loads bitstreams from the filesystem and assigns them to the
 * thread. A bitstream for each slot must be provided.
 *
 *   rt   - pointer to the ReconOS thread
 *   path - paths of the bitstream containing, %d replaced by slot number
 */
void reconos_thread_loadbitstream(struct reconos_thread *rt,
                                  char *path);

/*
 * Sets the main method of the software thread. When createing the
 * thread, a pointer to the thread is passed via the data parameter.
 *
 *   rt      - pointer to the ReconOS thread
 *   swentry - function pointer to the main entry point
 */
void reconos_thread_setswentry(struct reconos_thread *rt,
                               void *(*swentry)(void *data));

/*
 * Creates the ReconOS thread and executes it in the given slot number.
 *
 *   rt   - pointer to the ReconOS thread
 *   slot - slot number to execute the thread in
 *
 */
void reconos_thread_create(struct reconos_thread *rt, int slot);

/*
 * Creates the ReconOS thread and executes it in a free slot.
 *
 *   rt - pointer to the ReconOS thread
 *   tt - software or hardware thread
 *
 */
void reconos_thread_create_auto(struct reconos_thread *rt, int tt);

/*
 * Suspends the ReconOS thread by saving its state and pausing execution.
 * This method does not block until the thread is suspended, use
 * reconos_thread_join(...) to wait for termination of thread.
 *
 *   rt   - pointer to the ReconOS thread
 */
void reconos_thread_suspend(struct reconos_thread *rt);

/*
 * Suspends the ReconOS thread by saving its state and pausing execution.
 * This method blocks unit the thread is suspended.
 */
void reconos_thread_suspend_block(struct reconos_thread *rt);

/*
 * Resumes the ReconOS thread in the given slot by restoring its state
 * and starting execution.
 *
 *   rt   - pointer to the ReconOS thread
 *   slot - slot number to execute the thread in
 */
void reconos_thread_resume(struct reconos_thread *rt, int slot);

/*
 * Waits for the termination of the hardware thread.
 *
 *   rt - pointer to the ReconOS thread
 */
void reconos_thread_join(struct reconos_thread *rt);

/*
 * Sets a signal to the hardware thread. The signal must be cleared
 * by the hardware using the right system call.
 *
 *   rt - pointer to the ReconOS thread
 */
void reconos_thread_signal(struct reconos_thread *rt);

/*
 * Sets the priority of the thread (for rt systems)
 * 
 *
 *   rt - pointer to the ReconOS thread
 * 	thread_priority - priority
 */
void reconos_thread_setpriority(struct reconos_thread * rt, int thread_priority);

/* == General functions ================================================ */

/*
 * Initializes the ReconOS environment and resets the hardware. You must
 * call this method before you can use any of the other functions.
 */
void reconos_init();

/*
 * Cleans up the ReconOS environment and resets the hardware. You should
 * call this method before termination to prevent the hardware threads from
 * operating and avoid undesirable effects.
 * By default this method is registered as a signal handler for SIGINT,
 * SIGTERM and SIGABRT. Keep this in mind when overriding these handlers.
 */
void reconos_cleanup();

/*
 * Flushes the cache if needed.
 */
void reconos_cache_flush();

#endif /* RECONOS_H */