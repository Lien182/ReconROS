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

<<reconos_preproc>>

#ifndef RECONOS_APP_H
#define RECONOS_APP_H

#include "mbox.h"
#include "ros.h"
#include "ros_pub.h"
#include "ros_sub.h"
#include "ros_service_server.h"
#include "ros_action_server.h"
#include "ros_service_client.h"
#include "ros_action_client.h"

#include <pthread.h>
#include <semaphore.h>

<<ROSMsgHeader>>

/* == Application resources ============================================ */

/*
 * Definition of different resources of the application.
 *
 *   mbox  - mailbox (struct mbox)
 *   sem   - semaphore (sem_t)
 *   mutex - mutex (pthread_mutex)
 *   cond  - condition variable (pthread_cond)
 */
<<generate for RESOURCES(Type == "mbox")>>
extern struct mbox <<NameLower>>_s;
extern struct mbox *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "sem")>>
extern sem_t <<NameLower>>_s;
extern sem_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "mutex")>>
extern pthread_mutex_t <<NameLower>>_s;
extern pthread_mutex_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "cond")>>
extern pthread_cond_t <<NameLower>>_s;
extern pthread_cond_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rosnode")>>
extern struct ros_node_t <<NameLower>>_s;
extern struct ros_node_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rossub")>>
extern struct ros_subscriber_t <<NameLower>>_s;
extern struct ros_subscriber_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rospub")>>
extern struct ros_publisher_t <<NameLower>>_s;
extern struct ros_publisher_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rossrvs")>>
extern struct ros_service_server_t <<NameLower>>_s;
extern struct ros_service_server_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rosactions")>>
extern struct ros_action_server_t <<NameLower>>_s;
extern struct ros_action_server_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rossrvc")>>
extern struct ros_service_client_t <<NameLower>>_s;
extern struct ros_service_client_t *<<NameLower>>;
<<end generate>>

<<generate for RESOURCES(Type == "rosactionc")>>
extern struct ros_action_client_t <<NameLower>>_s;
extern struct ros_action_client_t *<<NameLower>>;
<<end generate>>


<<generate for RESOURCES(Type == "rosmsg" or Type == "rossrvmsgreq" or Type == "rossrvmsgres" or Type == "rosactionmsggoalreq" or Type == "rosactionmsgresultres" or Type == "rosactionmsgfeedback")>>
extern <<ROSDataType>> <<NameLower>>_s;
extern <<ROSDataType>> *<<NameLower>>;
<<end generate>>

/* == Application functions ============================================ */

/*
 * Initializes the application by creating all resources.
 */
void reconos_app_init();

/*
 * Cleans up the application by destroying all resources.
 */
void reconos_app_cleanup();

<<generate for THREADS>>
<<=generate for HasHw=>>
/*
 * Creates a hardware thread in the specified slot with its associated
 * resources.
 *
 *   rt   - pointer to the ReconOS thread
 */
struct reconos_thread *reconos_thread_create_hwt_<<Name>>(void * init_data);

struct reconos_thread *reconos_thread_pr_create_hwt_<<Name>>(void * init_data, char * bitstream_path);
<<=end generate=>>





<<=generate for HasSw=>>
/*
 * Creates a software thread with its associated resources.
 *
 *   rt   - pointer to the ReconOS thread
 */
struct reconos_thread *reconos_thread_create_swt_<<Name>>(void * init_data, int priority);
<<=end generate=>>

/*
 * Destroyes a hardware thread created.
 *
 *   rt   - pointer to the ReconOS thread
 */
void reconos_thread_destroy_<<Name>>(struct reconos_thread *rt);
<<end generate>>

<<generate for CLOCKS>>
/*
 * Sets the frequency for the iven clock. Returns the actual clock which
 * were able to configure for the clock.
 *
 *   f - the wanted frequency in kHz
 */
int reconos_clock_<<NameLower>>_set(int f);
<<end generate>>

#endif /* RECONOS_APP_H */