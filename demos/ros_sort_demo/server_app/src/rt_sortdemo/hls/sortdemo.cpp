#include "reconos_calls.h"
#include "reconos_thread.h"
#include "ap_cint.h"

#include <sorter_msgs/msg/sortdata.h>

#define BLOCK_SIZE 2048


void sort_bubble(uint32 ram[BLOCK_SIZE]) {
	unsigned int i, j;
	uint32 tmp;
	for (i = 0; i < BLOCK_SIZE; i++) {
		for (j = 0; j < BLOCK_SIZE - 1; j++) {
			if (ram[j] > ram[j + 1]) {
				tmp = ram[j];
				ram[j] = ram[j + 1];
				ram[j + 1] = tmp;
			}
		}
	}
}

void sort_net(uint32 ram[BLOCK_SIZE]) {
	unsigned int i, k, stage;
	uint32 tmp;

	for(stage = 1; stage <= BLOCK_SIZE; stage++){
		k = (stage % 2 == 1) ? 0 : 1;
		for(i = k; i < BLOCK_SIZE - 1; i += 2){
			if (ram[i] > ram[i + 1]) {
				tmp = ram[i];
				ram[i] = ram[i + 1];
				ram[i + 1] = tmp;
			}
		}
	}
}

THREAD_ENTRY() {
	RAM(uint32, BLOCK_SIZE, ram);
	uint32 addr, initdata;	
	uint32 pMessage;
	uint32 payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		pMessage = ROS_SUBSCRIBE_TAKE(resources_subdata, resources_sort_msg );
		addr = OFFSETOF(sorter_msgs__msg__Sortdata, sortdata.data) + pMessage;

		MEM_READ(addr, payload_addr, 4);					//Get the address of the data
		MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

		sort_bubble(ram);
		MEM_WRITE(ram, payload_addr[0], BLOCK_SIZE * 4);
		
		ROS_PUBLISH(resources_pubdata,resources_sort_msg);
	}
}
