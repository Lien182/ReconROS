/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Application library
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Auto-generated application specific header file
 *                 including definitions of all resources and functions
 *                 to instantiate resources and threads automatically.
 *
 * ======================================================================
 */



#ifndef RECONOS_APP_H
#define RECONOS_APP_H

#include "mbox.h"

#include <pthread.h>
#include <semaphore.h>

/* == Application resources ============================================ */

/*
 * Definition of different resources of the application.
 *
 *   mbox  - mailbox (struct mbox)
 *   sem   - semaphore (sem_t)
 *   mutex - mutex (pthread_mutex)
 *   cond  - condition variable (pthread_cond)
 */
extern struct mbox resources_address_s;
extern struct mbox *resources_address;

extern struct mbox resources_acknowledge_s;
extern struct mbox *resources_acknowledge;










/* == Application functions ============================================ */

/*
 * Initializes the application by creating all resources.
 */
void reconos_app_init();

/*
 * Cleans up the application by destroying all resources.
 */
void reconos_app_cleanup();

/*
 * Creates a hardware thread in the specified slot with its associated
 * resources.
 *
 *   rt   - pointer to the ReconOS thread
 */
struct reconos_thread *reconos_thread_create_hwt_sortdemo();


/*
 * Creates a software thread with its associated resources.
 *
 *   rt   - pointer to the ReconOS thread
 */
struct reconos_thread *reconos_thread_create_swt_sortdemo();


/*
 * Destroyes a hardware thread created.
 *
 *   rt   - pointer to the ReconOS thread
 */
void reconos_thread_destroy_sortdemo(struct reconos_thread *rt);



/*
 * Sets the frequency for the iven clock. Returns the actual clock which
 * were able to configure for the clock.
 *
 *   f - the wanted frequency in kHz
 */
int reconos_clock_system_set(int f);

/*
 * Sets the frequency for the iven clock. Returns the actual clock which
 * were able to configure for the clock.
 *
 *   f - the wanted frequency in kHz
 */
int reconos_clock_threads_set(int f);



#endif /* RECONOS_APP_H */