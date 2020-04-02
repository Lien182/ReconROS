/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Private header file
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Head file with private only definitions.
 *
 * ======================================================================
 */

#ifndef RECONOS_PRIVATE_H
#define RECONOS_PRIVATE_H

#include "reconos.h"
#include "utils.h"

#include <stdint.h>
#include <semaphore.h>


/* == ReconOS proc control ============================================= */

/*
 * Handles page faults from the memory subsystem
 *
 *   arg - null
 */
void *proc_pgfhandler(void *arg);


/* == ReconOS hwslot =================================================== */

/*
 * Definition of the delegate states.
 *
 *   stoped          - delegate not running
 *   init            - initializing
 *   blocked_osif    - blocked in reading from osif
 *   blocked_syscall - blocked in syscall
 *   processing      - processing a request
 */
#define DELEGATE_STATE_STOPPED          0x01
#define DELEGATE_STATE_INIT             0x02
#define DELEGATE_STATE_BLOCKED_OSIF     0x04
#define DELEGATE_STATE_BLOCKED_SYSCALL  0x08
#define DELEGATE_STATE_PROCESSING       0x10

/*
 * Definition of signals to the delegate thread.
 *
 *   pause_syscalls - pauses incoming syscalls by not sending result
 *   suspend        - requested suspend of thread
 */
#define DELEGATE_FLAG_PAUSE_SYSCALLS 0x01
#define DELEGATE_FLAG_SUSPEND 0x02

/*
 * Object representing a hardware slot on the FPGA.
 *
 *   id        - global id as used in the hardware design
 *
 *   osif      - file descriptor of the osif
 *
 *   rt        - pointer to the currently executing threads
 *   dt        - reference to the delegate thread
 *   dt_state  - state of the delegate thread
 *   dt_flags  - flags to the delegate thread
 *   dt_exit   - semaphore for synchronizing with the delegate on exit
 */
struct hwslot {
	int id;

	int osif;

	struct reconos_thread *rt;
	pthread_t dt;
	int dt_state;
	int dt_flags;
	sem_t dt_exit;
};

/*
 * Initializes the slot.
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_init(struct hwslot *slot, int id, int osif);

/*
 * Reset the slot.
 *
 *   slot  - pointer to the ReconOS slot
 */
void hwslot_reset(struct hwslot *slot);

/*
 * Sets the reset of the slot.
 *
 *   slot  - pointer to the ReconOS slot
 *   reset - zero or one to set the reset
 */
void hwslot_setreset(struct hwslot *slot, int reset);

/*
 * Sets the signal of the slot.
 *
 *   slot  - pointer to the ReconOS slot
 *   reset - zero or one to set the signal
 */
void hwslot_setsignal(struct hwslot *slot, int sig);

/*
 * Creates a new delegate thread if not present.
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_createdelegate(struct hwslot *slot);

/*
 * Stops the delegate thread at an appropriate point in time
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_stopdelegate(struct hwslot *slot);

/*
 * Executes the given ReconOS thread in the slot by resetting
 * the hardware and reconfiguring it if needed. Running threads
 * will be killed or an error occurs.
 *
 *   slot - pointer to the ReconOS slot
 *   rt   - pointer to the ReconOS thread
 */
void hwslot_createthread(struct hwslot *slot,
                         struct reconos_thread *rt);

/*
 * Suspends the active thread by saving its state and termination the
 * delegate thread.
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_suspendthread(struct hwslot *slot);

/*
 * Resumes the thread by restoring its state. Running threads will be
 * killed or an error occurs.
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_resumethread(struct hwslot *slot,
                         struct reconos_thread *rt);

/*
 * Waits for the termination of the running thread and resets the thread
 * afterwards.
 *
 *   slot - pointer to the ReconOS slot
 */
void hwslot_jointhread(struct hwslot *slot);


/* == ReconOS delegate ================================================= */

/*
 * Definition of the osif commands
 *
 *   self-describing
 *
 */
#define OSIF_CMD_THREAD_GET_INIT_DATA  0x000000A0
#define OSIF_CMD_THREAD_GET_STATE_ADDR 0x000000A1
#define OSIF_CMD_THREAD_EXIT           0x000000A2
#define OSIF_CMD_THREAD_YIELD          0x000000A3
#define OSIF_CMD_THREAD_CLEAR_SIGNAL   0x000000A4
#define OSIF_CMD_SEM_POST              0x000000B0
#define OSIF_CMD_SEM_WAIT              0x000000B1
#define OSIF_CMD_MUTEX_LOCK            0x000000C0
#define OSIF_CMD_MUTEX_UNLOCK          0x000000C1
#define OSIF_CMD_MUTEX_TRYLOCK         0x000000C2
#define OSIF_CMD_COND_WAIT             0x000000D0
#define OSIF_CMD_COND_SIGNAL           0x000000D1
#define OSIF_CMD_COND_BROADCAST        0x000000D2
#define OSIF_CMD_MBOX_GET              0x000000F0
#define OSIF_CMD_MBOX_PUT              0x000000F1
#define OSIF_CMD_MBOX_TRYGET           0x000000F2
#define OSIF_CMD_MBOX_TRYPUT           0x000000F3
#define OSIF_CMD_MASK                  0x000000FF
#define OSIF_CMD_YIELD_MASK            0x80000000

#define OSIF_SIGNAL_THREAD_START       0x01000000
#define OSIF_SIGNAL_THREAD_RESUME      0x01000001

#define OSIF_INTERRUPTED               0x000000FF

/*
 * Global method of the delegate thread
 *
 *   arg - pointer to the ReconOS hwslot
 */
void *dt_delegate(void *arg);

#endif /* RECONOS_PRIVATE_H */