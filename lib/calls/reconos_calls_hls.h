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

<<reconos_preproc>>

#ifndef RECONOS_CALLS_H
#define RECONOS_CALLS_H

#include "hls_stream.h"
#include "stdint.h"
#include "ap_int.h"

#include "ap_axi_sdata.h"
//#include <sensor_msgs/msg/image.h>

<<generate for HWTOPICSPUB>>
#include <<<include>>>
<<end generate>>

<<generate for HWTOPICSSUB>>
#include <<<include>>>
<<end generate>>

/* == Helper definitions =============================================== */

#define RRBASETYPE 			<<RRBASETYPE>>
#define RRUBASETYPE			<<RRUBASETYPE>>
#define RRBASETYPEBYTES		<<RRBASETYPEBYTES>>

#if (RRBASETYPEBYTES == 8)
    #define RRUBASETYPEMAX 0xFFFFFFFFFFFFFFFF
#elif (RRBASETYPEBYTES == 4)
    #define RRUBASETYPEMAX 0xFFFFFFFF
#endif

#define OFFSETOF(type, member) ((RRUBASETYPE)(intptr_t)&(((type *)(void*)0)->member) )

/* == Constant definitions ============================================= */

/*
 * General constants
 *
 *   MEMIF_CHUNK_WORDS - size of one memory request in words
 *                       (a request might be split up to meet this)
 */

typedef union
{
	uint64_t u64Adr;
	uint8_t* ptr;
}t_ptr_conversation;



#if RRBASETYPEBYTES == 4 

	#define MEMIF_CHUNK_WORDS 64
	#define MEMIF_CHUNK_BYTES (MEMIF_CHUNK_WORDS * 4)
	#define MEMIF_CHUNK_MASK  0x000000FF

	#define MEMIF_CMD_READ 0x00000000
	#define MEMIF_CMD_WRITE 0xF0000000

#elif RRBASETYPEBYTES == 8 
	#define MEMIF_CHUNK_WORDS 32
	#define MEMIF_CHUNK_BYTES (MEMIF_CHUNK_WORDS * 8)
	#define MEMIF_CHUNK_MASK  0x00000000000000FF

	#define MEMIF_CMD_READ 0x0000000000000000
	#define MEMIF_CMD_WRITE 0xF000000000000000
#endif
/*
 * Definition of the osif commands
 *
 *   self-describing
 *
 */
#define OSIF_CMD_THREAD_GET_INIT_DATA  			(RRUBASETYPE)0x000000A0
#define OSIF_CMD_THREAD_GET_STATE_ADDR 			(RRUBASETYPE)0x000000A1
#define OSIF_CMD_THREAD_EXIT           			(RRUBASETYPE)0x000000A2
#define OSIF_CMD_THREAD_YIELD          			(RRUBASETYPE)0x000000A3
#define OSIF_CMD_THREAD_CLEAR_SIGNAL   			(RRUBASETYPE)0x000000A4
#define OSIF_CMD_SEM_POST              			(RRUBASETYPE)0x000000B0
#define OSIF_CMD_SEM_WAIT              			(RRUBASETYPE)0x000000B1
#define OSIF_CMD_MUTEX_LOCK            			(RRUBASETYPE)0x000000C0
#define OSIF_CMD_MUTEX_UNLOCK          			(RRUBASETYPE)0x000000C1
#define OSIF_CMD_MUTEX_TRYLOCK         			(RRUBASETYPE)0x000000C2
#define OSIF_CMD_COND_WAIT             			(RRUBASETYPE)0x000000D0
#define OSIF_CMD_COND_SIGNAL           			(RRUBASETYPE)0x000000D1
#define OSIF_CMD_COND_BROADCAST        			(RRUBASETYPE)0x000000D2
#define OSIF_CMD_MBOX_GET              			(RRUBASETYPE)0x000000F0
#define OSIF_CMD_MBOX_PUT              			(RRUBASETYPE)0x000000F1
#define OSIF_CMD_MBOX_TRYGET           			(RRUBASETYPE)0x000000F2
#define OSIF_CMD_MBOX_TRYPUT           			(RRUBASETYPE)0x000000F3
#define OSIF_CMD_MASK                  			(RRUBASETYPE)0x00000FFF
#define OSIF_CMD_YIELD_MASK            			(RRUBASETYPE)0x80000000

#define OSIF_SIGNAL_THREAD_START       			(RRUBASETYPE)0x01000000
#define OSIF_SIGNAL_THREAD_RESUME      			(RRUBASETYPE)0x01000001

#define OSIF_INTERRUPTED               			(RRUBASETYPE)0x000000FF

#define OSIF_CMD_ROS_MESSAGE_SET_SIZE			(RRUBASETYPE)0x000009F0

#define OSIF_CMD_MEMORY_MALLOC					(RRUBASETYPE)0x000000F4
#define OSIF_CMD_MEMORY_FREE					(RRUBASETYPE)0x000000F5
#define OSIF_CMD_MEMORY_GETOBJADDR				(RRUBASETYPE)0x000000F6
#define OSIF_CMD_MEMORY_GETMEMADDR				(RRUBASETYPE)0x000000F7

#define OSIF_CMD_ROS_PUBLISH		   			(RRUBASETYPE)0x00000900
#define OSIF_CMD_ROS_TAKE			   			(RRUBASETYPE)0x00000901
#define OSIF_CMD_ROS_TRYTAKE		   			(RRUBASETYPE)0x00000902
#define OSIF_CMD_ROS_TAKE_INTERRUPTABLE			(RRUBASETYPE)0x00000903
#define OSIF_CMD_ROS_TAKE_INTERRUPTABLE_CANCEL	(RRUBASETYPE)0x00000904

#define OSIF_CMD_ROS_SERVICES_RESPONSE 			(RRUBASETYPE)0x00000910
#define OSIF_CMD_ROS_SERVICES_TRYTAKE  			(RRUBASETYPE)0x00000911	
#define OSIF_CMD_ROS_SERVICES_TAKE 	   			(RRUBASETYPE)0x00000912

#define OSIF_CMD_ROS_ACTIONS_GOAL_TAKE			(RRUBASETYPE)0x00000920
#define OSIF_CMD_ROS_ACTIONS_GOAL_TRYTAKE		(RRUBASETYPE)0x00000921
#define OSIF_CMD_ROS_ACTIONS_GOAL_DECIDE		(RRUBASETYPE)0x00000922
#define OSIF_CMD_ROS_ACTIONS_RESULT_TAKE		(RRUBASETYPE)0x00000923
#define OSIF_CMD_ROS_ACTIONS_RESULT_TRYTAKE		(RRUBASETYPE)0x00000924
#define OSIF_CMD_ROS_ACTIONS_RESULT_SEND		(RRUBASETYPE)0x00000925
#define OSIF_CMD_ROS_ACTIONS_FEEDBACK			(RRUBASETYPE)0x00000926
#define OSIF_CMD_ROS_ACTIONS_CANCEL_TAKE		(RRUBASETYPE)0x00000927
#define OSIF_CMD_ROS_ACTIONS_CANCEL_TRYTAKE		(RRUBASETYPE)0x00000928
#define OSIF_CMD_ROS_ACTIONS_CANCEL_SEND		(RRUBASETYPE)0x00000929

//Clients
#define OSIF_CMD_ROS_SERVICEC_REQUEST 			(RRUBASETYPE)0x00000930
#define OSIF_CMD_ROS_SERVICEC_TRYTAKE  			(RRUBASETYPE)0x00000931	
#define OSIF_CMD_ROS_SERVICEC_TAKE 	   			(RRUBASETYPE)0x00000932

#define OSIF_CMD_ROS_ACTIONC_GOAL_SEND			(RRUBASETYPE)0x00000940
#define OSIF_CMD_ROS_ACTIONC_GOAL_TRYTAKE		(RRUBASETYPE)0x00000941
#define OSIF_CMD_ROS_ACTIONC_GOAL_TAKE			(RRUBASETYPE)0x00000942
#define OSIF_CMD_ROS_ACTIONC_RESULT_SEND		(RRUBASETYPE)0x00000943
#define OSIF_CMD_ROS_ACTIONC_RESULT_TAKE		(RRUBASETYPE)0x00000944
#define OSIF_CMD_ROS_ACTIONC_RESULT_TRYTAKE		(RRUBASETYPE)0x00000945

#define OSIF_CMD_ROS_ACTIONC_FEEDBACK_TAKE		(RRUBASETYPE)0x00000946
#define OSIF_CMD_ROS_ACTIONC_FEEDBACK_TRYTAKE	(RRUBASETYPE)0x00000947

#define OSIF_CMD_ROS_ACTIONC_CANCEL_SEND		(RRUBASETYPE)0x00000948
#define OSIF_CMD_ROS_ACTIONC_CANCEL_TAKE		(RRUBASETYPE)0x00000949
#define OSIF_CMD_ROS_ACTIONC_CANCEL_TRYTAKE		(RRUBASETYPE)0x00000950


/*
 * Definition of memif commands
 *
 *   self-describing
 */



/* == Internal functions =============================================== */

/*
 * Writes blocking to a stream using the non-blocking method. Since the
 * non-blocking write is called in a loop, vivado hls enforces sequential
 * order, which is necessary for osif and memif calls.
 *
 *   stream - reference to stream
 *   data   - data to write
 */
inline void stream_write(hls::stream<RRUBASETYPE> &stream, RRUBASETYPE data) {
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
inline RRUBASETYPE stream_read(hls::stream<RRUBASETYPE> &stream, hls::stream<RRUBASETYPE> &outputstream, volatile bool &hwt_signal) {
#pragma HLS inline
	RRUBASETYPE data;
	while (!stream.read_nb(data)){ if(hwt_signal){ stream_write(outputstream, OSIF_CMD_THREAD_EXIT);while(1);}}
	return data;
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
inline RRUBASETYPE stream_read_memif(hls::stream<RRUBASETYPE> &stream) {
#pragma HLS inline
	RRUBASETYPE data;
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
 	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal);

/*
 * Posts the semaphore specified by handle.
 *
 *   @see sem_post
 */
#define SEM_POST(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_SEM_POST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Waits for the semaphore specified by handle.
 *
 *   @see sem_wait
 */
#define SEM_WAIT(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_SEM_WAIT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Locks the mutex specified by handle.
 *
 *   @see pthread_mutex_lock
 */
#define MUTEX_LOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_LOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Unlocks the mutex specified by handle.
 *
 *   @see pthread_mutex_unlock
 */
#define MUTEX_UNLOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_UNLOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Tries to lock the mutex specified by handle and returns if successful or not.
 *
 *   @see pthread_mutex_trylock
 */
#define MUTEX_TRYLOCK(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MUTEX_TRYLOCK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Waits for the condition variable specified by handle.
 *
 *   @see pthread_cond_wait
 */
#define COND_WAIT(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_WAIT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Signals a single thread waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_signal
 */
#define COND_SIGNAL(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_SIGNAL),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Signals all threads waiting on the condition variable specified by handle.
 *
 *   @see pthread_cond_broadcast
 */
#define COND_BROADCAST(p_handle,p_handle2)(\
	stream_write(osif_hw2sw, OSIF_CMD_COND_BROADCAST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Puts a single word into the mbox specified by handle.
 *
 *   @see mbox_get
 */
#define MBOX_GET(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_GET),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Reads a single word from the mbox specified by handle.
 *
 *   @see mbox_put
 */
#define MBOX_PUT(p_handle,data)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_PUT),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, data),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 * Tries to put a single word into the mbox specified by handle but does not
 * blocks until the mbox gets populated.
 *
 *   @see mbox_tryget
 */
#define MBOX_TRYGET(p_handle,data)(\
	stream_write(osif_hw2sw, OSIF_CMD_MBOX_TRYGET),\
	stream_write(osif_hw2sw, p_handle),\
	data = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

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
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

/*
 *	Memory functions
 */

#define MEMORY_GETOBJECTADDR(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_GETOBJADDR),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define MEMORY_GETMEMORYADDR(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_GETMEMADDR),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define MEMORY_MALLOC(ptr_dest,length)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_MALLOC),\
	stream_write(osif_hw2sw, ptr_dest),\
	stream_write(osif_hw2sw, length),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define MEMORY_FREE(ptr)(\
	stream_write(osif_hw2sw, OSIF_CMD_MEMORY_FREE),\
	stream_write(osif_hw2sw, ptr))

/************************************************************************************
 * 
 * ROS Extensions 
 * 
 * *********************************************************************************/


#define ROS_MESSAGE_ARRAY_SET_SIZE(p_handle, offset, element_size, size)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_MESSAGE_SET_SIZE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, offset),\
	stream_write(osif_hw2sw, element_size),\
	stream_write(osif_hw2sw, size),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))


//ROS Communication function


#define ROS_PUBLISH(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_PUBLISH),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SUBSCRIBE_TRYTAKE(p_handle,p_handle_msg, msg_ptr)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	msg_ptr = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SUBSCRIBE_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw, osif_hw2sw, hwt_signal))

#define ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_START(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_TAKE_INTERRUPTABLE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg))

#define ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_CHECK(msg_ptr)(\
		osif_sw2hw.read_nb(msg_ptr))

#define ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_CANCEL(msg_ptr, ret)({\
	if(osif_sw2hw.read_nb(msg_ptr)){ \
		ret = 1; } \
	else{ \
		stream_write(osif_hw2sw, OSIF_CMD_ROS_TAKE_INTERRUPTABLE_CANCEL),\
		msg_ptr = stream_read(osif_sw2hw, osif_hw2sw, hwt_signal); \
		if(msg_ptr != RRUBASETYPEMAX) \
			ret = 1; \
		else \
			ret = 0;\
		}})

// Extensions for hardware topics
typedef ap_axis<64,1,1,1> t_stream;


// HWTopics with shifting
#define HWTOPIC_AXIS_DWIDTH 64
#define ARRAY_DWIDTH 8
#define UNROLL_FACTOR HWTOPIC_AXIS_DWIDTH/ARRAY_DWIDTH


<<generate for HWTOPICSPUB>>
#define ROS_PUBLISH_HWTOPIC_PACKED_<<Name>>( <<Name>>, msg){\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	write_section_<<Name>> : { \
	_Pragma ("HLS protocol fixed") \
	__tmp_frame.user = 0; \
	__tmp_frame.last = 0; \
	__tmp_frame.data = <<HWPUBLISHERID>> + 1000; \
	(<<Name>>).write(__tmp_frame); \
	__tmp_frame.user = 1; \
	(<<Name>>).write(__tmp_frame); \
	<<=generate for Primitives=>>
		__tmp_frame.data = (msg).<<name>>; \
		(<<Name>>).write(__tmp_frame);  <<=end generate=>> <<=generate for Arrays_8bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>>} \
		__tmp_frame.user = 0; \
		__tmp_frame.last = 1; \
		(<<Name>>).write(__tmp_frame); \
}
<<end generate>>

<<generate for HWTOPICSPUB>>
#define ROS_PUBLISH_HWTOPIC_PACKED_SIZE_<<Name>>( <<Name>>, msg){\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	write_section_<<Name>> : { \
	_Pragma ("HLS protocol fixed") \
	__tmp_frame.user = 0; \
	__tmp_frame.last = 0; \
	(<<Name>>).write(__tmp_frame); \
	__tmp_frame.user = 1; \
	(<<Name>>).write(__tmp_frame); \
	<<=generate for Primitives=>>
		__tmp_frame.data = (msg).<<name>>; \
		(<<Name>>).write(__tmp_frame);  <<=end generate=>> <<=generate for Arrays_8bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size/8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size/4; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size/2; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size/1; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>>} \
		__tmp_frame.user = 0; \
		__tmp_frame.last = 1; \
		(<<Name>>).write(__tmp_frame); \
}
<<end generate>>

<<generate for HWTOPICSPUB>>
#define ROS_PUBLISH_HWTOPIC_PACKED_EXPERIMENT_<<Name>>( <<Name>>, msg){\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	uint64_t __cnt = 0; \
	write_section_<<Name>> : { \
	_Pragma ("HLS protocol fixed") \
	__tmp_frame.user = 0; \
	__tmp_frame.last = 0; \
	(<<Name>>).write(__tmp_frame); \
	__tmp_frame.user = 1; \
	(<<Name>>).write(__tmp_frame); \
	<<=generate for Primitives=>>
		__tmp_frame.data = (msg).<<name>>; \
		(<<Name>>).write(__tmp_frame);  <<=end generate=>> <<=generate for Arrays_8bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < ((msg).<<name>>.size + 7)/8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = __cnt++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < ((msg).<<name>>.size + 3)/4; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = __cnt++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < ((msg).<<name>>.size + 1)/2; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = __cnt++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = __cnt++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>>} \
		__tmp_frame.user = 0; \
		__tmp_frame.last = 1; \
		(<<Name>>).write(__tmp_frame); \
}
<<end generate>>

<<generate for HWTOPICSSUB>>
#define ROS_CLEAR_HWTOPIC_PACKED_<<Name>>( <<Name>>){\
	ap_axis<64,1,1,1> __tmp_frame; \
	size_t __arraysize = 0; \
	(<<Name>>).read(__tmp_frame); \
	<<=generate for Primitives=>>
		(<<Name>>).read(__tmp_frame); \
		<<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); __arraysize = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < (__arraysize + 7)/ 8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> \
		(<<Name>>).read(__tmp_frame); \
}
<<end generate>>

<<generate for HWTOPICSSUB>>
#define ROS_CLEAR_HWTOPIC_PACKED_SIZE_<<Name>>( <<Name>>){\
	ap_axis<64,1,1,1> __tmp_frame; \
	size_t __arraysize = 0; \
	(<<Name>>).read(__tmp_frame); \
	(<<Name>>).read(__tmp_frame); \
	<<=generate for Primitives=>>
		(<<Name>>).read(__tmp_frame); \
		<<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); __arraysize = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < (__arraysize + 7)/ 8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
		} \<<=end generate=>> \
		(<<Name>>).read(__tmp_frame); \
}
<<end generate>>

<<generate for HWTOPICSSUB>>
#define ROS_READ_HWTOPIC_PACKED_<<Name>>( <<Name>>, msg){\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	(<<Name>>).read(__tmp_frame); \
	(<<Name>>).read(__tmp_frame); \
	<<=generate for Primitives=>>
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>> = __tmp_frame.data;  <<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> \
		(<<Name>>).read(__tmp_frame); \
}
<<end generate>>


<<generate for HWTOPICSSUB>>
#define ROS_READ_HWTOPIC_PACKED_SIZE_<<Name>>( <<Name>>, msg){\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	(<<Name>>).read(__tmp_frame); \
	(<<Name>>).read(__tmp_frame); \
	<<=generate for Primitives=>>
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>> = __tmp_frame.data;  <<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size / 8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size / 4; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size / 2; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.size = __tmp_frame.data; \
		(<<Name>>).read(__tmp_frame); \
		(msg).<<name>>.capacity = __tmp_frame.data; \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < (msg).<<name>>.size; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			(<<Name>>).read(__tmp_frame); \
			*__dataptr = __tmp_frame.data; __dataptr++;\
		} \<<=end generate=>> \
		(<<Name>>).read(__tmp_frame); \
}
<<end generate>>


<<generate for HWTOPICSSUB>>
#define ROS_TRYREAD_HWTOPIC_PACKED_<<Name>>( <<Name>>, msg, retval)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	if(!(<<Name>>).empty()) { \
		(<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		<<=generate for Primitives=>>
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>> = __tmp_frame.data;  <<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				*__dataptr = __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				*__dataptr = __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				*__dataptr = __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				*__dataptr = __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> \
			(<<Name>>).read(__tmp_frame); \
			retval = 1UL;\
	} else {retval = 0UL; } \
})
<<end generate>>

<<generate for HWTOPICSSUB>>
#define ROS_TRYREAD_HWTOPIC_PACKED_EXPERIMENT_<<Name>>( <<Name>>, msg, accu, retval)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t * __dataptr; \
	if(!(<<Name>>).empty()) { \
		(<<Name>>).read(__tmp_frame); \
		(<<Name>>).read(__tmp_frame); \
		<<=generate for Primitives=>>
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>> = __tmp_frame.data;  <<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < ((msg).<<name>>.size + 7)/ 8; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				(accu) += __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < ((msg).<<name>>.size + 3)/ 4; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				(accu) += __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < ((msg).<<name>>.size + 1)/ 2;i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				(accu) +=  __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.size = __tmp_frame.data; \
			(<<Name>>).read(__tmp_frame); \
			(msg).<<name>>.capacity = __tmp_frame.data; \
			__dataptr = (uint64_t *)((msg).<<name>>.data);\
			for (uint32_t i = 0; i < (msg).<<name>>.size; i++) \
			{ \
				_Pragma ("HLS pipeline") \
				_Pragma ("HLS UNROLL factor=1") \
				(<<Name>>).read(__tmp_frame); \
				(accu) += __tmp_frame.data; __dataptr++;\
			} \<<=end generate=>> \
			(<<Name>>).read(__tmp_frame); \
			retval = 1UL;\
	} else {retval = 0UL; } \
})
<<end generate>>


<<generate for HWTOPICSSUB>>
#define ROS_TRYREAD_HWTOPIC_PACKED_TO_MEMORY_NEW_<<Name>>( <<Name>>, msg, msg_ptr, retval)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint64_t __inputbuf[MEMIF_CHUNK_WORDS]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	uint64_t __payload[1]; \
	uint32_t __rem; \
	if(!(<<Name>>).empty()) { \
		(<<Name>>).read(__tmp_frame); \
		if(__tmp_frame.data != (<<HWPUBLISHERID>> + 1000)){ \
			(<<Name>>).read(__tmp_frame); \
			<<=generate for Primitives=>>
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>> = __tmp_frame.data; \
				*(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]) = (msg).<<name>>; \<<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]) = (msg).<<name>>.size;\
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]) = (msg).<<name>>.capacity;\
				MEM_READ(OFFSETOF(<<datatype>>, <<name>>.data) + msg_ptr, __payload, 8); \
				*(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]) = __payload[0];\
				__dataptr = __payload[0];\
				for (int32_t __rem = (int32_t)((msg).<<name>>.size + 7)/ 8; __rem > 0; __rem-=MEMIF_CHUNK_WORDS){ \
					_Pragma ("HLS DATAFLOW") \
					uint32_t __writesize = (__rem >= MEMIF_CHUNK_WORDS) ? MEMIF_CHUNK_WORDS : __rem; \
					for(uint32_t j = 0; j < __writesize; j++){ \
						_Pragma ("HLS pipeline") \
						_Pragma ("HLS UNROLL factor=1") \
						(<<Name>>).read(__tmp_frame); \
						__inputbuf[j] = __tmp_frame.data;\
					} \
					MEM_WRITE_FUNC(memif_hwt2mem, memif_mem2hwt, __dataptr, __inputbuf, __writesize); \
					__dataptr += __writesize*8; \
				} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> \
				(<<Name>>).read(__tmp_frame); \
				MEM_WRITE(__buf, msg_ptr,  ((sizeof(<<datatype>>) + 7) / 8) * 8);\
				retval = 1UL;\
		} else {\
			ROS_CLEAR_HWTOPIC_PACKED_<<Name>>( <<Name>>); \
			retval = 0UL; \
		}; \
	} else {retval = 0UL; } \
})
<<end generate>>

<<generate for HWTOPICSSUB>>
#define ROS_TRYREAD_HWTOPIC_PACKED_TO_MEMORY_<<Name>>( <<Name>>, msg, msg_ptr, retval)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	uint64_t __payload[1]; \
	if(!(<<Name>>).empty()) { \
		(<<Name>>).read(__tmp_frame); \
		if(__tmp_frame.data != (<<HWPUBLISHERID>> + 1000)){ \
			(<<Name>>).read(__tmp_frame); \
			<<=generate for Primitives=>>
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>> = __tmp_frame.data; \
				*(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]) = (msg).<<name>>; \<<=end generate=>> <<=generate for Arrays_8bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]) = (msg).<<name>>.size;\
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]) = (msg).<<name>>.capacity;\
				MEM_READ(OFFSETOF(<<datatype>>, <<name>>.data) + msg_ptr, __payload, 8); \
				*(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]) = __payload[0];\
				__dataptr = __payload[0];\
				for (uint32_t i = 0; i < ((msg).<<name>>.size + 7)/ 8; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					__payload[0] = __tmp_frame.data; \
					MEM_WRITE(__payload, __dataptr, 8 );\
					__dataptr += 8;\
				} \<<=end generate=>> <<=generate for Arrays_16bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> <<=generate for Arrays_32bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> <<=generate for Arrays_64bit=>> (<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.size = __tmp_frame.data; \
				(<<Name>>).read(__tmp_frame); \
				(msg).<<name>>.capacity = __tmp_frame.data; \
				__dataptr = (uint64_t *)((msg).<<name>>.data);\
				for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
				{ \
					_Pragma ("HLS pipeline") \
					_Pragma ("HLS UNROLL factor=1") \
					(<<Name>>).read(__tmp_frame); \
					*__dataptr = __tmp_frame.data; __dataptr++;\
				} \<<=end generate=>> \
				(<<Name>>).read(__tmp_frame); \
				MEM_WRITE(__buf, msg_ptr,  ((sizeof(<<datatype>>) + 7) / 8) * 8);\
				retval = 1UL;\
		} else {\
			ROS_CLEAR_HWTOPIC_PACKED_<<Name>>( <<Name>>); \
			retval = 0UL; \
		}; \
	} else {retval = 0UL; } \
})
<<end generate>>

<<generate for HWTOPICSPUB>>
#define ROS_PUBLISH_HWTOPIC_PACKED_FROM_MEMORY_<<Name>>( <<Name>>, msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8];\
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	uint64_t __payload[1]; \
	write_section_<<Name>>:{ \
	MEM_READ(msg_ptr, __buf,  ((sizeof(<<datatype>>) + 7) / 8) * 8);\
	__tmp_frame.user = 0; \
	__tmp_frame.last = 0; \
	__tmp_frame.data = <<HWPUBLISHERID>> + 1000; \
	(<<Name>>).write(__tmp_frame); \
	__tmp_frame.user = 1; \
	(<<Name>>).write(__tmp_frame); \
	<<=generate for Primitives=>>
		(msg).<<name>> = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]); \
		__tmp_frame.data = (msg).<<name>>; \
		(<<Name>>).write(__tmp_frame);\<<=end generate=>> <<=generate for Arrays_8bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
		__tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]);\
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
		for (uint32_t i = 0; i < ((msg).<<name>>.size + 7)/ 8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			MEM_READ(__dataptr, __payload, 8); \
			__tmp_frame.data = __payload[0]; __dataptr+=8; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>).write(__tmp_frame); \
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>).write(__tmp_frame); \
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>).write(__tmp_frame); \
		} \<<=end generate=>>} \
		__tmp_frame.user = 0; \
		__tmp_frame.last = 1; \
		(<<Name>>).write(__tmp_frame); \
})
<<end generate>>


<<generate for HWTOPICSPUB>>
#define ROS_PUBLISH_HWTOPIC_PACKED_FROM_MEMORY_AND_CLEAR_<<Name>>( <<Name>>_in, <<Name>>_out, msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8];\
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	uint64_t __payload[1]; \
	write_section_<<Name>>:{ \
	MEM_READ(msg_ptr, __buf,  ((sizeof(<<datatype>>) + 7) / 8) * 8);\
	__tmp_frame.user = 0; \
	__tmp_frame.last = 0; \
	__tmp_frame.data = <<HWPUBLISHERID>> + 1000; \
	(<<Name>>_in).write(__tmp_frame); \
	(<<Name>>_out).read(__tmp_frame); \
	__tmp_frame.user = 1; \
	(<<Name>>_in).write(__tmp_frame); \
	(<<Name>>_out).read(__tmp_frame); \
	<<=generate for Primitives=>>
		(msg).<<name>> = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]); \
		__tmp_frame.data = (msg).<<name>>; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame); \<<=end generate=>> <<=generate for Arrays_8bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
		__tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame); \
		(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]);\
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
		for (uint32_t i = 0; i < ((msg).<<name>>.size + 7)/ 8; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			MEM_READ(__dataptr, __payload, 8); \
			__tmp_frame.data = __payload[0]; __dataptr+=8; \
			(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame); \
		} \<<=end generate=>> <<=generate for Arrays_16bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		} \<<=end generate=>> <<=generate for Arrays_32bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		} \<<=end generate=>> <<=generate for Arrays_64bit=>> __tmp_frame.data = (msg).<<name>>.size;  \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__tmp_frame.data = (msg).<<name>>.capacity; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		__dataptr = (uint64_t *)((msg).<<name>>.data);\
		for (uint32_t i = 0; i < <<NameUpper>>_<<num_elems>>; i++) \
		{ \
			_Pragma ("HLS pipeline") \
			_Pragma ("HLS UNROLL factor=1") \
			__tmp_frame.data = *__dataptr; __dataptr++; \
			(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
		} \<<=end generate=>>} \
		__tmp_frame.user = 0; \
		__tmp_frame.last = 1; \
		(<<Name>>_in).write(__tmp_frame); (<<Name>>_out).read(__tmp_frame);\
})
<<end generate>>

<<generate for ROSMSG>>
#define MEM_READ_ROS_MESSAGE_<<datatype>>(msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	MEM_READ(msg_ptr, __buf, ((sizeof(<<datatype>>) + 7) / 8) * 8); \
	<<=generate for Primitives=>>(msg).<<name>> = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]); \
	<<=end generate=>><<=generate for Arrays_8bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	MEM_READ(__dataptr,(uint64_t*)((msg).<<name>>.data),(((msg).<<name>>.size + 7) / 8) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_16bit=>> (msg).<<name>>.size = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	MEM_READ(__dataptr,(uint64_t*)((msg).<<name>>.data)(((msg).<<name>>.size + 3) / 4) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_32bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	MEM_READ(__dataptr,((uint64_t*)((msg).<<name>>.data),(((msg).<<name>>.size + 1) / 2) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_64bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	MEM_READ(__dataptr, (msg).<<name>>.data,(msg).<<name>>.size); \<<=end generate=>>})
<<end generate>>

<<generate for ROSMSG>>
#define MEM_READ_ROS_MESSAGE_EXPERIMENT_<<datatype>>(msg_ptr, msg, result)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __accu = 0; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	MEM_READ(msg_ptr, __buf, ((sizeof(<<datatype>>) + 7) / 8) * 8); \
	<<=generate for Primitives=>>(msg).<<name>> = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]); \
	<<=end generate=>><<=generate for Arrays_8bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	if(__dataptr != 0) MEM_READ_EXPERIMENT(__dataptr,__accu,(((msg).<<name>>.size + 7) / 8) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_16bit=>> (msg).<<name>>.size = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	if(__dataptr != 0) MEM_READ_EXPERIMENT(__dataptr,__accu,(((msg).<<name>>.size + 3) / 4) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_32bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	if(__dataptr != 0) MEM_READ_EXPERIMENT(__dataptr,__accu,(((msg).<<name>>.size + 1) / 2) * 8 ); \
	\<<=end generate=>><<=generate for Arrays_64bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]);\
	if(__dataptr != 0) MEM_READ_EXPERIMENT(__dataptr,__accu,(msg).<<name>>.size); \<<=end generate=>>result = __accu;})
<<end generate>>

<<generate for ROSMSG>>
#define MEM_INIT_ROS_MESSAGE_<<datatype>>(msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame; \
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t * __dataptr; \
	t_ptr_conversation ptr_conversation; \
	MEM_READ(msg_ptr, __buf, ((sizeof(<<datatype>>) + 7) / 8) * 8); \
	<<=generate for Primitives=>>(msg).<<name>> = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]); \
	<<=end generate=>><<=generate for Arrays_8bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	ptr_conversation.u64Adr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]); \
	(msg).<<name>>.data = (<<dtype>>)ptr_conversation.ptr; \
	\<<=end generate=>><<=generate for Arrays_16bit=>> (msg).<<name>>.size = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	ptr_conversation.u64Adr = *(uint64_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]); \
	(msg).<<name>>.data = ptr_conversation.ptr; \
	\<<=end generate=>><<=generate for Arrays_32bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	(msg).<<name>>.data = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]); \
	\<<=end generate=>><<=generate for Arrays_64bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	(msg).<<name>>.data = *(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]); \<<=end generate=>>})
<<end generate>>

<<generate for ROSMSG>>
#define MEM_WRITE_ROS_MESSAGE_<<datatype>>(msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame;\
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t __dataptr; \
	MEM_READ(msg_ptr, __buf, ((sizeof(<<datatype>>) + 7) / 8) * 8); \
	<<=generate for Primitives=>>*(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]) = (msg).<<name>>; \
	<<=end generate=>><<=generate for Arrays_8bit=>> *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]) = (msg).<<name>>.size;\
	*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]) = (msg).<<name>>.capacity; \
	__dataptr = *(uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE((uint64_t*)((msg).<<name>>.data),__dataptr,(((msg).<<name>>.size + 7) / 8) * 8 ); \<<=end generate=>>\
	<<=generate for Arrays_16bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE((msg).<<name>>.data,__dataptr,(((msg).<<name>>.size + 3) / 4) * 8 );\<<=end generate=>>\
	<<=generate for Arrays_32bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE((msg).<<name>>.data,__dataptr,(((msg).<<name>>.size + 1) / 2) * 8 );\<<=end generate=>>\
	<<=generate for Arrays_64bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE((msg).<<name>>.data,__dataptr,(msg).<<name>>.size);\ <<=end generate=>>\
	MEM_WRITE(__buf, msg_ptr,  ((sizeof(<<datatype>>) + 7) / 8) * 8); })
<<end generate>>

<<generate for ROSMSG>>
#define MEM_WRITE_ROS_MESSAGE_EXPERIMENT_<<datatype>>(msg_ptr, msg)({\
	ap_axis<64,1,1,1> __tmp_frame;\
	uint64_t __buf[(sizeof(<<datatype>>) + 7) / 8]; \
	uint8_t * __bufu8ptr = (uint8_t*)__buf; \
	uint64_t * __dataptr; \
	uint64_t __cnt = 0; \
	MEM_READ(msg_ptr, __buf, ((sizeof(<<datatype>>) + 7) / 8) * 8); \
	<<=generate for Primitives=>>*(<<dtype>>*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>)]) = (msg).<<name>>; \
	<<=end generate=>><<=generate for Arrays_8bit=>> *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]) = (msg).<<name>>.size;\
	*(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]) = (msg).<<name>>.capacity; \
	__dataptr = (uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE_EXPERIMENT(__cnt,__dataptr[0],(((msg).<<name>>.size + 7) / 8) * 8 ); \<<=end generate=>>\
	<<=generate for Arrays_16bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = (uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE_EXPERIMENT(__cnt,__dataptr[0],(((msg).<<name>>.size + 3) / 4) * 8 );\<<=end generate=>>\
	<<=generate for Arrays_32bit=>>(msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = *(uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE_EXPERIMENT(__cnt,__dataptr[0],(((msg).<<name>>.size + 1) / 2) * 8 );\<<=end generate=>>\
	<<=generate for Arrays_64bit=>> (msg).<<name>>.size = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.size)]);\
	(msg).<<name>>.capacity = *(size_t*)(&__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.capacity)]); \
	__dataptr = (uint64_t*)(&(__bufu8ptr[OFFSETOF(<<datatype>>, <<name>>.data)]));\
	MEM_WRITE_EXPERIMENT(__cnt,__dataptr[0],(msg).<<name>>.size);\ <<=end generate=>>\
	MEM_WRITE(__buf, msg_ptr,  ((sizeof(<<datatype>>) + 7) / 8) * 8); })
<<end generate>>

// ROS Services

#define ROS_SERVICESERVER_SEND_RESPONSE(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_RESPONSE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SERVICESERVER_TRYTAKE(p_handle,p_handle_msg, msg_ptr)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	msg_ptr = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal), \
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SERVICESERVER_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICES_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SERVICECLIENT_SEND_REQUEST(p_handle,p_handle_msg)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_REQUEST),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SERVICECLIENT_TRYTAKE(p_handle,p_handle_msg ,msg_ptr)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	msg_ptr = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal), \
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_SERVICECLIENT_TAKE(p_handle, p_handle_msg )(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_SERVICEC_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))




//ROS Actions

#define ROS_ACTION_SERVER_GOAL_REJECT       0
#define ROS_ACTION_SERVER_GOAL_ACCEPT       1


#define ROS_ACTIONSERVER_GOAL_TAKE(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_GOAL_TRYTAKE(p_handle,p_handle_msg, msg_ptr)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	msg_ptr = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal), \
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))


#define ROS_ACTIONSERVER_GOAL_DECIDE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_GOAL_DECIDE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, accept),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_RESULT_TAKE(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_RESULT_TRYTAKE(p_handle, msg_ptr)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	msg_ptr = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal), \
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_RESULT_SEND(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_RESULT_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_FEEDBACK(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_FEEDBACK),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_CANCEL_TAKE(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_CANCEL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_CANCEL_TRYTAKE(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_CANCEL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONSERVER_CANCEL_SEND(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONS_CANCEL_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTION_CLIENT_GOAL_REJECTED       0
#define ROS_ACTION_CLIENT_GOAL_ACCEPTED       1


#define ROS_ACTIONCLIENT_GOAL_SEND(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_GOAL_TRYTAKE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	accept = stream_read(osif_sw2hw, osif_hw2sw, hwt_signal),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_GOAL_TAKE(p_handle,accept)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_GOAL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	accept = stream_read(osif_sw2hw,osif_hw2sw, hwt_signal),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal)) 

#define ROS_ACTIONCLIENT_RESULT_SEND(p_handle)(\
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_RESULT_TRYTAKE(p_handle, p_handle_msg)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_RESULT_TAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_RESULT_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_FEEDBACK_TAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_FEEDBACK_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_FEEDBACK_TRYTAKE(p_handle, p_handle_msg )( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_FEEDBACK_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_write(osif_hw2sw, p_handle_msg),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))


#define ROS_ACTIONCLIENT_CANCEL_SEND(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_CANCEL_SEND),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_CANCEL_TRYTAKE(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_CANCEL_TRYTAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

#define ROS_ACTIONCLIENT_CANCEL_TAKE(p_handle)( \
	stream_write(osif_hw2sw, OSIF_CMD_ROS_ACTIONC_CANCEL_TAKE),\
	stream_write(osif_hw2sw, p_handle),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal)) 

/*
 * Gets the pointer to the initialization data of the ReconOS thread
 * specified by reconos_hwt_setinitdata.
 */
#define GET_INIT_DATA()(\
	stream_write(osif_hw2sw, OSIF_CMD_THREAD_GET_INIT_DATA),\
	stream_read(osif_sw2hw,osif_hw2sw, hwt_signal))

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
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (src), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			(dst)[__i++] = memif_mem2hwt.read();\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}}

#define MEM_READ_EXPERIMENT(src,accu,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (src), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			(accu) += memif_mem2hwt.read();\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}}

/*
* Can read byte-wise from int8/uint8 and write to int8/uint8
*/
#define MEM_READ_INT8(src,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (src), __i = 0;\
	uint64_t __tmp = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			__tmp = memif_mem2hwt.read();\
            for(uint8_t __k = 0; __k < 8; __k++ ) { \
                (dst)[__i++] = __tmp & 0xff;\
                __tmp = __tmp >> 8;}\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
			__tmp = 0;\
		}\
	}}

#define MEM_READ_INT32(src,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (src), __i = 0;\
	uint64_t __tmp = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
			_Pragma ("HLS pipeline")  \
			__tmp = memif_mem2hwt.read();\
			(dst)[__i++] = __tmp & 0xffffffff;\
			__tmp = __tmp >> 32; \
			(dst)[__i++] = __tmp & 0xffffffff;\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
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
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			memif_hwt2mem.write((src)[__i++]);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}}

#define MEM_WRITE_EXPERIMENT(cnt,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			memif_hwt2mem.write((cnt)++);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}}

/*
* Can read byte-wise from memory (int8/uint8) and write to local ram (int8/uint8)
*/
#define MEM_WRITE_INT8_REVERSED(src,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	uint64_t __temp; \
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
			__temp = 0;\
			for(uint8_t __k = 0; __k < 8; __k++){\
				__temp = __temp | (0xff & (src)[__i + __k]);\
				if(__k < 7){\
				__temp = __temp << 8;}\
			}\
		_Pragma ("HLS pipeline")  \
			memif_hwt2mem.write(__temp);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
			__i += 8;\
		}\
	}}

#define MEM_WRITE_INT8(src,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	uint64_t __temp; \
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
			__temp = 0;\
			for(int8_t __k = 7; __k >= 0; __k--){\
				__temp = __temp | (0xff & (src)[__i + __k]);\
				if(__k > 0){\
				__temp = __temp << 8;}\
			}\
		_Pragma ("HLS pipeline")  \
			memif_hwt2mem.write(__temp);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
			__i += 8;\
		}\
	}}

#define MEM_WRITE_INT32(src,dst,len){\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	uint64_t __temp; \
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
			__temp = 0;\
			__temp = __temp | (0xffffffff & (src)[__i]);\
			__temp = __temp << 32; \
			__temp = __temp | (0xffffffff & (src)[__i+1]);\
		_Pragma ("HLS pipeline")  \
			memif_hwt2mem.write(__temp);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
			__i += 2;\
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


#define MEM_READ_TO_STREAM( src, dst, len){ \
	RRUBASETYPE __len, __rem; \
	RRUBASETYPE __addr = (src), __i = 0; \
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_READ | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			dst.write(memif_mem2hwt.read());\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}\
}

#define MEM_WRITE_FROM_STREAM( src, dst, len)\
{\
	RRUBASETYPE __len, __rem;\
	RRUBASETYPE __addr = (dst), __i = 0;\
	for (__rem = (len); __rem > 0;) {\
		RRUBASETYPE __to_border = MEMIF_CHUNK_BYTES - (__addr & MEMIF_CHUNK_MASK);\
		RRUBASETYPE __to_rem = __rem;\
		if (__to_rem < __to_border)\
			__len = __to_rem;\
		else\
			__len = __to_border;\
		\
		stream_write(memif_hwt2mem, MEMIF_CMD_WRITE | __len);\
		stream_write(memif_hwt2mem, __addr);\
		\
		for (; __len > 0; __len -= RRBASETYPEBYTES) {\
		_Pragma ("HLS pipeline")  \
			RRUBASETYPE tmp = src.read();\
			memif_hwt2mem.write(tmp);\
			__addr += RRBASETYPEBYTES;\
			__rem -= RRBASETYPEBYTES;\
		}\
	}\
}

/*
 * Terminates the current ReconOS thread.
 */
#define THREAD_EXIT(){\
	stream_write(osif_hw2sw, OSIF_CMD_THREAD_EXIT);\
	while(1);}


void MEM_WRITE_FUNC(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, uint64_t destptr,  uint64_t * buf, uint32_t size)
{
	MEM_WRITE(buf, destptr, size*8 );
}


#endif /* RECONOS_CALLS_H */
