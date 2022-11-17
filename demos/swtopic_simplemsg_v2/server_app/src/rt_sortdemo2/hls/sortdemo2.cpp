#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>
#include <std_msgs/msg/u_int64_multi_array__struct.h>

#define BLOCK_SIZE 2048


THREAD_ENTRY() {
	RAM(uint64_t, BLOCK_SIZE/2, ram);
	uint64_t addr, initdata;	
	uint64_t pMessage;
	uint64_t payload_addr[1];
	uint64_t mbox_value = 0;

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		io_section :{
		#pragma HLS protocol fixed
		pMessage = ROS_SUBSCRIBE_TAKE(resources2_subdata, resources2_sort_msg );
		//addr = OFFSETOF(std_msgs__msg__UInt32MultiArray, data.data) + pMessage;
		addr = OFFSETOF(std_msgs__msg__UInt64MultiArray, data.data) + pMessage;

		MEM_READ(addr, payload_addr, 8);	//Get the address of the data
		MEM_READ(payload_addr[0], ram, 8);

		MBOX_PUT(resources2_finish_mbox, ram[0]);
		}
	}
}