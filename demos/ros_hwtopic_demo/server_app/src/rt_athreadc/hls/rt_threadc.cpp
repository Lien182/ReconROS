#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

t_stream tmpdata;

THREAD_ENTRY() {

//	#pragma HLS INTERFACE axis port=nicehwtopic

	RAM(uint32_t, BLOCK_SIZE, ram);
	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint32_t payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		for(int i = 0; i < BLOCK_SIZE; i++)
		{
//			tmpdata = nicehwtopic.read();
			ram[i] = tmpdata.data;
		}
			
		//sort_bubble(ram);
		MEM_WRITE(ram, payload_addr[0], BLOCK_SIZE * 4);		
		//ROS_PUBLISH(rthreadc_pubdata, rthreadc_sort_msg);
	}
}
