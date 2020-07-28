/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Calls library header file
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  ReconOS calls to allow interaction with POSIX-API of
 *                 system calls.
 *
 * ======================================================================
 */

#ifndef RECONOS_CALLS_H
#define RECONOS_CALLS_H

#include "mbox.h"

#include "ros_pub.h"
#include "ros_sub.h"
#include "ros_service_server.h"
#include "ros_action_server.h"
#include "ros_service_client.h"

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

/* == Call functions =================================================== */

/*
 * Creates a local ram to be used for mem functions. You may only pass
 * rams created by this macro to mem functions.
 *
 *   type - datatype of the ram 
 *   size - size of the ram
 *   name - name of the ram
 */
#define RAM(type,size,name)\
 	type *name

/*
 * Initializes the thread and reads from the osif the resume status.
 */
#define THREAD_INIT()

/*
 * Posts the semaphore specified by handle.
 *
 *   @see sem_post
 */
#define SEM_POST(p_handle)\
	sem_post((p_handle))

/*
 * Waits for the semaphore specified by handle.
 *
 *   @see sem_wait
 */
#define SEM_WAIT(p_handle)\
	sem_wait((p_handle))

/*
 * Locks the mutex specified by handle.
 *
 *   @see pthread_mutex_lock
 */
#define MUTEX_LOCK(p_hande)\
	pthread_mutex_lock((p_handle))

/*
 * Unlocks the mutex specified by handle.
 *
 *   @see pthread_mutex_unlock
 */
#define MUTEX_UNLOCK(p_handle)\
	pthread_mutex_unlock(p_handle)

/*
 * Tries to lock the mutex specified by handle and returns if successful or not.
 *
 *   @see pthread_mutex_trylock
 */
#define MUTEX_TRYLOCK(p_handle)\
	pthread_mutex_trylock((p_handle))

/*
 * Waits for the condition variable specified by handle.
 *
 *   @see pthread_cond_wait
 */
#define COND_WAIT(p_handle,p_handle2)\
	pthread_cond_wait((p_handle), (p_handle2))

/*
 * Signals a single thread waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_signal
 */
#define COND_SIGNAL(p_handle,p_handle2)\
	pthread_cond_signal((p_handle), (p_handle2))

/*
 * Signals all threads waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_broadcast
 */
#define COND_BROADCAST(p_handle,p_handle2)\
	pthread_cond_broadcast((p_handle), (p_handle2))

/*
 * Puts a single word into the mbox specified by handle.
 *
 *   @see mbox_get
 */
#define MBOX_GET(p_handle)\
	mbox_get((p_handle))

/*
 * Reads a single word from the mbox specified by handle.
 *
 *   @see mbox_put
 */
#define MBOX_PUT(p_handle,data)\
	mbox_put((p_handle), (data))

/*
 * Tries to put a single word into the mbox specified by handle but does not
 * blocks until the mbox gets populated.
 *
 *   @see mbox_tryget
 */
#define MBOX_TRYGET(p_handle,data)\
	mbox_tryget((p_handle), (&data))

/*
 * Tries to read a single word from the mbox specified by handle but does not
 * blocks until the mbox gets free.
 *
 *   @see mbox_tryput
 */
#define MBOX_TRYPUT(p_handle,data)\
	mbox_tryput((p_handle), (data))

/*
 * Gets the pointer to the initialization data of the ReconOS thread
 * specified by reconos_hwt_setinitdata.
 */
#define GET_INIT_DATA()\
	data

/*
 * Reads several words from the main memory into the local ram.
 *
 *   src - start address to read from the main memory
 *   dst - array to write data into
 *   len - number of bytes to transmit (bytes)
 */
#define MEM_READ(src,dst,len)\
	dst = (void *)src

/*
 * Writes several words from the local ram into the main memory.
 *
 *   src - array to read data from
 *   dst - start address to read from the main memory
 *   len - number of bytes to transmit (bytes)
 */
#define MEM_WRITE(src, dst, len)

/*
 * Terminates the current ReconOS thread.
 */
#define THREAD_EXIT()\
	pthread_exit(0);


/*
 * ROS functions
 */

#define ROS_PUBLISH(p_handle, p_handle_msg) \
	ros_publisher_publish(p_handle, p_handle_msg);

#define ROS_SUBSCRIBE_TRYTAKE(p_handle,p_handle_msg) \
	ros_subscriber_message_try_take(p_handle, p_handle_msg);

#define ROS_SUBSCRIBE_TAKE(p_handle, p_handle_msg ) \
	ros_subscriber_message_take(p_handle, p_handle_msg);


#define ROS_SERVICESERVER_SEND_RESPONSE(p_handle, p_handle_msg) \
	ros_service_server_response_send(p_handle, p_handle_msg);

#define ROS_SERVICESERVER_TRYTAKE(p_handle,p_handle_msg) \
	ros_service_server_request_try_take(p_handle, p_handle_msg);

#define ROS_SERVICESERVER_TAKE(p_handle, p_handle_msg ) \
	ros_service_server_request_take(p_handle, p_handle_msg);


#define ROS_SERVICECLIENT_SEND(p_handle, p_handle_msg) \
	ros_service_client_request_send(p_handle, p_handle_msg);

#define ROS_SERVICECLIENT_TRYTAKE(p_handle,p_handle_msg) \
	ros_service_client_response_try_take(p_handle, p_handle_msg);

#define ROS_SERVICECLIENT_TAKE(p_handle, p_handle_msg ) \
	ros_service_client_response_take(p_handle, p_handle_msg);


#define ROS_ACTIONSERVER_GOAL_TAKE(p_handle, p_handle_msg) \
	ros_action_server_goal_take(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONSERVER_GOAL_TRYTAKE(p_handle,p_handle_msg) \
	ros_action_server_goal_try_take(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONSERVER_GOAL_DECIDE(p_handle,accept) \
	ros_action_server_goal_decide(p_handle, accept);

#define ROS_ACTIONSERVER_RESULT_TAKE(p_handle) \
	ros_action_server_result_take(p_handle);

#define ROS_ACTIONSERVER_RESULT_TRYTAKE(p_handle) \
	ros_action_server_result_try_take(p_handle);

#define ROS_ACTIONSERVER_RESULT_SEND(p_handle, p_handle_msg ) \
	ros_action_server_result_send(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONSERVER_FEEDBACK(p_handle, p_handle_msg ) \
	ros_action_server_feedback(p_handle, (void*)p_handle_msg);


#define ROS_ACTIONCLIENT_GOAL_SEND(p_handle, p_handle_msg) \
	ros_action_client_goal_send(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONCLIENT_GOAL_TAKE(p_handle,accept) \
	ros_action_client_goal_take(p_handle, accept);

#define ROS_ACTIONCLIENT_GOAL_TRYTAKE(p_handle,p_handle_msg) \
	ros_action_client_goal_try_take(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONCLIENT_RESULT_SEND(p_handle ) \
	ros_action_client_result_send(p_handle);

#define ROS_ACTIONCLIENT_RESULT_TAKE(p_handle,p_handle_msg) \
	ros_action_client_result_take(p_handle,(void*)p_handle_msg);

#define ROS_ACTIONCLIENT_RESULT_TRYTAKE(p_handle,p_handle_msg) \
	ros_action_client_result_try_take(p_handle,(void*)p_handle_msg);

#define ROS_ACTIONCLIENT_FEEDBACK_TAKE(p_handle, p_handle_msg ) \
	ros_action_client_feedback_take(p_handle, (void*)p_handle_msg);

#define ROS_ACTIONCLIENT_FEEDBACK_TRYTAKE(p_handle, p_handle_msg ) \
	ros_action_client_feedback_try_take(p_handle, (void*)p_handle_msg);
#endif /* RECONOS_CALLS_H */