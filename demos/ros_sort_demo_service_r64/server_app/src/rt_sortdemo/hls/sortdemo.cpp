#include "reconos_calls.h"
#include "reconos_thread.h"

#include <sorter_msgs/srv/sort.h>

#define BLOCK_SIZE 2048


void sort_net(uint32_t * ram) {
	unsigned int i, k, stage;
	uint32_t tmp;
	uint32_t prev_val;
	uint32_t next_val;

	for(stage = 1; stage <= BLOCK_SIZE; stage++){
		k = (stage % 2 == 1) ? 0 : 1;
		for(i = k; i < BLOCK_SIZE - 1; i += 2){
			#pragma HLS unroll factor=16
			#pragma HLS pipeline
			ram[i] = prev_val;
			ram[i + 1] = next_val;
			if (prev_val > ram[i + 1]) {
				ram[i] = next_val;
				ram[i + 1] = prev_val;
			}
		}
	}
}

THREAD_ENTRY() {
	RAM(uint32_t, BLOCK_SIZE, ram);
	uint64_t addr, initdata;	
	uint32_t pMessage;
	uint32_t payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		pMessage = ROS_SERVICESERVER_TAKE(resources_srv, resources_sort_srv_req );
		addr = OFFSETOF(sorter_msgs__srv__Sort_Request, unsorted.data) + pMessage;

		MEM_READ(addr, payload_addr, 4);					//Get the address of the data
		MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

		sort_net((uint32_t *)ram);
		MEM_WRITE(ram, payload_addr[0], BLOCK_SIZE * 4);
		
		ROS_SERVICESERVER_SEND_RESPONSE(resources_srv,resources_sort_srv_res);
	}
}
