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

#include "hls_stream.h"
#include "ap_cint.h"

/* == Helper definitions =============================================== */

#define OFFSETOF(type, member) ((uint32)(intptr_t)&(((type *)(void*)0)->member) )

/* == Constant definitions ============================================= */

/*
 * General constants
 *
 *   MEMIF_CHUNK_WORDS - size of one memory request in words
 *                       (a request might be split up to meet this)
 */
#define MEMIF_CHUNK_WORDS 64
#define MEMIF_CHUNK_BYTES (MEMIF_CHUNK_WORDS * 4)
#define MEMIF_CHUNK_MASK  0x000000FF

/*
 * Definition of the osif commands
 *
 *   self-describing
 *
 */
#define OSIF_CMD_THREAD_GET_INIT_DATA  			0x000000A0
#define OSIF_CMD_THREAD_GET_STATE_ADDR 			0x000000A1
#define OSIF_CMD_THREAD_EXIT           			0x000000A2
#define OSIF_CMD_THREAD_YIELD          			0x000000A3
#define OSIF_CMD_THREAD_CLEAR_SIGNAL   			0x000000A4
#define OSIF_CMD_SEM_POST              			0x000000B0
#define OSIF_CMD_SEM_WAIT              			0x000000B1
#define OSIF_CMD_MUTEX_LOCK            			0x000000C0
#define OSIF_CMD_MUTEX_UNLOCK          			0x000000C1
#define OSIF_CMD_MUTEX_TRYLOCK         			0x000000C2
#define OSIF_CMD_COND_WAIT             			0x000000D0
#define OSIF_CMD_COND_SIGNAL           			0x000000D1
#define OSIF_CMD_COND_BROADCAST        			0x000000D2
#define OSIF_CMD_MBOX_GET              			0x000000F0
#define OSIF_CMD_MBOX_PUT              			0x000000F1
#define OSIF_CMD_MBOX_TRYGET           			0x000000F2
#define OSIF_CMD_MBOX_TRYPUT           			0x000000F3
#define OSIF_CMD_MASK                  			0x000000FF
#define OSIF_CMD_YIELD_MASK            			0x80000000

#define OSIF_SIGNAL_THREAD_START       			0x01000000
#define OSIF_SIGNAL_THREAD_RESUME      			0x01000001

#define OSIF_INTERRUPTED               			0x000000FF

#define OSIF_CMD_ROS_MESSAGE_SET_SIZE			0x00000950

#define OSIF_CMD_ROS_PUBLISH		   			0x00000900
#define OSIF_CMD_ROS_TAKE			   			0x00000901
#define OSIF_CMD_ROS_TRYTAKE		   			0x00000902

#define OSIF_CMD_ROS_SERVICES_RESPONSE 			0x00000910
#define OSIF_CMD_ROS_SERVICES_TRYTAKE  			0x00000911	
#define OSIF_CMD_ROS_SERVICES_TAKE 	   			0x00000912

#define OSIF_CMD_ROS_ACTIONS_GOAL_TAKE			0x00000920
#define OSIF_CMD_ROS_ACTIONS_GOAL_TRYTAKE		0x00000921
#define OSIF_CMD_ROS_ACTIONS_GOAL_DECIDE		0x00000922
#define OSIF_CMD_ROS_ACTIONS_RESULT_TAKE		0x00000923
#define OSIF_CMD_ROS_ACTIONS_RESULT_TRYTAKE		0x00000924
#define OSIF_CMD_ROS_ACTIONS_RESULT_SEND		0x00000925
#define OSIF_CMD_ROS_ACTIONS_FEEDBACK			0x00000926

//Clients
#define OSIF_CMD_ROS_SERVICEC_REQUEST 			0x00000930
#define OSIF_CMD_ROS_SERVICEC_TRYTAKE  			0x00000931	
#define OSIF_CMD_ROS_SERVICEC_TAKE 	   			0x00000932

#define OSIF_CMD_ROS_ACTIONC_GOAL_SEND			0x00000940
#define OSIF_CMD_ROS_ACTIONC_GOAL_TRYTAKE		0x00000941
#define OSIF_CMD_ROS_ACTIONC_GOAL_TAKE			0x00000942
#define OSIF_CMD_ROS_ACTIONC_RESULT_SEND		0x00000943
#define OSIF_CMD_ROS_ACTIONC_RESULT_TAKE		0x00000944
#define OSIF_CMD_ROS_ACTIONC_RESULT_TRYTAKE		0x00000945

#define OSIF_CMD_ROS_ACTIONC_FEEDBACK_TAKE		0x00000946
#define OSIF_CMD_ROS_ACTIONC_FEEDBACK_TRYTAKE	0x00000947


/*
 * Definition of memif commands
 *
 *   self-describing
 */
#define MEMIF_CMD_READ 0x00000000
#define MEMIF_CMD_WRITE 0xF0000000


/* == Internal functions =============================================== */

/*
 * Writes blocking to a stream using the non-blocking method. Since the
 * non-blocking write is called in a loop, vivado hls enforces sequential
 * order, which is necessary for osif and memif calls.
 *
 *   stream - reference to stream
 *   data   - data to write
 */
inline void stream_write(hls::stream<uint32> &stream, uint32 data) {
#pragma HLS inline
	while (!stream.write_nb(data)){}
}

/*
 * Reads blocking from a stream using the non-blocking method. Since the
 * non-blocking read is called in a loop, vivado hls enforces sequential
 * order, which is necessary for osif and memif calls.
 *
 *   stream - reference to stream
 *
 *   @returns read data
 */
inline uint32 stream_read(hls::stream<uint32> &stream) {
#pragma HLS inline
	uint32 data;
	while (!stream.read_nb(data)){}
	return data;
}

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
	type name[size]

/*
 * Initializes the thread and reads from the osif the resume status.
 */
#define THREAD_INIT()\
 	stream_read(osif_sw2hw)

/*
 * Posts the semaphore specified by handle.
 *
 *   @see sem_post
 */
#define SEM_POST(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_SEM_POST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Waits for the semaphore specified by handle.
 *
 *   @see sem_wait
 */
#define SEM_WAIT(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_SEM_WAIT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Locks the mutex specified by handle.
 *
 *   @see pthread_mutex_lock
 */
#define MUTEX_LOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_LOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Unlocks the mutex specified by handle.
 *
 *   @see pthread_mutex_unlock
 */
#define MUTEX_UNLOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_UNLOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Tries to lock the mutex specified by handle and returns if successful or not.
 *
 *   @see pthread_mutex_trylock
 */
#define MUTEX_TRYLOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_TRYLOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Waits for the condition variable specified by handle.
 *
 *   @see pthread_cond_wait
 */
#define COND_WAIT(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_WAIT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Signals a single thread waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_signal
 */
#define COND_SIGNAL(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_SIGNAL),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Signals all threads waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_broadcast
 */
#define COND_BROADCAST(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_BROADCAST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Puts a single word into the mbox specified by handle.
 *
 *   @see mbox_get
 */
#define MBOX_GET(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_GET),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

/*
 * Reads a single word from the mbox specified by handle.
 *
 *   @see mbox_put
 */
#define MBOX_PUT(p_handle,data)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_PUT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, data),\
	stream_read(osif_sw2hw))

/*
 * Tries to put a single word into the mbox specified by handle but does not
 * blocks until the mbox gets populated.
 *
 *   @see mbox_tryget
 */
#define MBOX_TRYGET(p_handle,data)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_TRYGET),\
	stream_write(osif_hw2sw, p_handle),\
	data = stream_read(osif_sw2hw),\
	stream_read(osif_sw2hw))

/*
 * Tries to read a single word from the mbox specified by handle but does not
 * blocks until the mbox gets free.
 *
 *   @see mbox_tryput
 */
#define MBOX_TRYPUT(p_handle,data)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_TRYPUT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, data),\
	stream_read(osif_sw2hw))



#define MEMORY_MALLOC(ptr_dest,length)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_MALLOC),\
	stream_write(osif_hw2sw, ptr_dest),\
	stream_write(osif_hw2sw, length),\
	stream_read(osif_sw2hw))

#define MEMORY_FREE(ptr)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_FREE),\
	stream_write(osif_hw2sw, ptr))

/************************************************************************************
 * 
 * ROS Extensions 
 * 
 * *********************************************************************************/


#define ROS_MESSAGE_ARRAY_SET_SIZE(p_handle, offset, size)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_MESSAGE_SET_SIZE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, offset),\
	stream_write(osif_hw2sw, size),\
	stream_read(osif_sw2hw))


//ROS Communication function


#define ROS_PUBLISH(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_PUBLISH),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SUBSCRIBE_TRYTAKE(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SUBSCRIBE_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))


// ROS Services

#define ROS_SERVICESERVER_SEND_RESPONSE(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_RESPONSE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SERVICESERVER_TRYTAKE(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SERVICESERVER_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))



#define ROS_SERVICECLIENT_SEND_REQUEST(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_REQUEST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SERVICECLIENT_TRYTAKE(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_SERVICECLIENT_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))




//ROS Actions

#define ROS_ACTION_SERVER_GOAL_REJECT       0
#define ROS_ACTION_SERVER_GOAL_ACCEPT       1


#define ROS_ACTIONSERVER_GOAL_TAKE(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_GOAL_TRYTAKE(p_handle,p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_GOAL_DECIDE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_DECIDE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, accept),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_RESULT_TAKE(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_RESULT_TRYTAKE(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_RESULT_SEND(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONSERVER_FEEDBACK(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_FEEDBACK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))


#define ROS_ACTION_CLIENT_GOAL_REJECTED       0
#define ROS_ACTION_CLIENT_GOAL_ACCEPTED       1


#define ROS_ACTIONCLIENT_GOAL_SEND(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_GOAL_TRYTAKE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	accept = stream_read(osif_hw2sw),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_GOAL_TAKE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	accept = stream_read(osif_hw2sw),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_RESULT_SEND(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_RESULT_TRYTAKE(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_RESULT_TAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_FEEDBACK_TAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_FEEDBACK_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

#define ROS_ACTIONCLIENT_FEEDBACK_TRYTAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_FEEDBACK_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw))

/*
 * Gets the pointer to the initialization data of the ReconOS thread
 * specified by reconos_hwt_setinitdata.
 */
#define GET_INIT_DATA()(\
	stream_write(osif_hw2sw, OSIF_CMD_THREAD_GET_INIT_DATA),\
	stream_read(osif_sw2hw))

/*
 * Reads several words from the main memory into the local ram. Therefore,
 * divides a large request into smaller ones of length at most
 * MEMIF_CHUNK_BYTES and splits request at page borders to guarantee
 * correct address translation.
 *
 *   src - start address to read from the main memory
 *   dst - array to write data into
 *   len - number of bytes to transmit (bytes)
 *   
 */
#define MEM_READ(src,dst,len){\
	uint32 __len, __rem;\
	uint32 __addr = (src), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		uint32 __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		uint32 __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= 4) {\
			(dst)[__i++] = stream_read(memif_mem2hwt);\
			__addr += 4;\
			__rem -= 4;\
		}\
	}}

/*
 * Writes several words from the local ram into main memory. Therefore,
 * divides a large request into smaller ones of length at most
 * MEMIF_CHUNK_BYTES and splits request at page borders to guarantee
 * correct address translation.
 *
 *   src - array to read data from
 *   dst - start address to read from the main memory
 *   len - number of bytes to transmit (bytes)
 */
#define MEM_WRITE(src,dst,len){\
	uint32 __len, __rem;\
	uint32 __addr = (dst), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		uint32 __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		uint32 __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= 4) {\
			stream_write(memif_hwt2mem, (src)[__i++]);\
			__addr += 4;\
			__rem -= 4;\
		}\
	}}

/*
 * Writes several words from the local ram into main memory. Therefore,
 * divides a large request into smaller ones of length at most
 * MEMIF_CHUNK_BYTES and splits request at page borders to guarantee
 * correct address translation.
 *
 *   src - array to read data from
 *   dst - start address to read from the main memory
 *   len - number of bytes to transmit (bytes)
 */

#define MEM_WRITE_FROM_STREAM(src,dst,len){\
	uint32 __len = len; \
	stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
	stream_write(memif_hwt2mem, dst);\
	for (; __len > 0; __len -= 4) {\
		stream_write(memif_hwt2mem, src.read());\
	}\
	}


/*
 * Terminates the current ReconOS thread.
 */
#define THREAD_EXIT()(\
	pthread_exit(0))

#endif /* RECONOS_CALLS_H */
