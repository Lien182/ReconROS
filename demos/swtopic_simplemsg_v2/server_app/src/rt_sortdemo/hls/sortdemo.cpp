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

	uint64_t output_buffer_addr;
	output_buffer_addr = MEMORY_GETOBJECTADDR(resources_sort_msg);
	uint64_t mbox_value;

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {
		io_section : {
		#pragma HLS protocol fixed

		mbox_value = MBOX_GET(resources_start_mbox);

		//MEM_READ(OFFSETOF(std_msgs__msg__UInt32MultiArray, data.data) + output_buffer_addr, payload_addr, 8);
		MEM_READ(OFFSETOF(std_msgs__msg__UInt64MultiArray, data.data) + output_buffer_addr, payload_addr, 8);
		MEM_WRITE(ram, payload_addr[0], 8);
		
		ROS_PUBLISH(resources_pubdata,resources_sort_msg);
		}
	}
}