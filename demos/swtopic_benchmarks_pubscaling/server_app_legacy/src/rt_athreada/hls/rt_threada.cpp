#include "reconos_calls.h"
#include "reconos_thread.h"

#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

t_stream tmpdata;

THREAD_ENTRY() {

	RAM(uint32_t, BLOCK_SIZE, ram);
	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint32_t payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		pMessage = ROS_SUBSCRIBE_TAKE(rthreada_subdata, rthreada_sort_msg );
		addr = OFFSETOF(std_msgs__msg__UInt32MultiArray, data.data) + pMessage;

		MEM_READ(addr, payload_addr, 4);					//Get the address of the data
		MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

			
		//sort_bubble(ram);
		MEM_WRITE(ram, payload_addr[0], BLOCK_SIZE * 4);		
		ROS_PUBLISH(rthreada_pubdata, rthreada_sort_msg);
	}
}
