#include "reconos_calls.h"
#include "reconos_thread.h"
#include "ap_cint.h"

#include <sorter_msgs/action/sort.h>

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
		
		for(i = 0; i < BLOCK_SIZE - 1; i += 2){
			if (ram[i] > ram[i + 1]) {
				tmp = ram[i];
				ram[i] = ram[i + 1];
				ram[i + 1] = tmp;
			}
		}

		for(i = 1; i < BLOCK_SIZE - 1; i += 2){
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
	uint32 addr, initdata, accept = 0;	
	uint32 pMessage;
	uint32 payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();
	
	addr = ROS_MESSAGE_ARRAY_SET_SIZE(resources_sort_action_goal_req,  OFFSETOF(sorter_msgs__action__Sort_SendGoal_Request, goal.unsorted),  BLOCK_SIZE * 4);

	while(1) {

		for(int i = 0; i < BLOCK_SIZE; i++)
			ram[i] = BLOCK_SIZE - i;


		MEM_WRITE(ram, addr, BLOCK_SIZE * 4);

		ROS_ACTIONCLIENT_GOAL_SEND(resources_actionsclient, resources_sort_action_goal_req);		

		ROS_ACTIONCLIENT_GOAL_TAKE(resources_actionsclient, accept);
		if(accept)
		{

			ROS_ACTIONCLIENT_RESULT_SEND(resources_actionsclient);
						

			//Wait for response
			pMessage = ROS_ACTIONCLIENT_RESULT_TAKE(resources_actionsclient, resources_sort_action_result_res );
			pMessage += OFFSETOF(sorter_msgs__action__Sort_GetResult_Response, result.sorted.data);

			MEM_READ(pMessage, payload_addr, 4);					//Get the address of the data
			MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

			uint32 sorted = 1;

			for(int i = 1; i < BLOCK_SIZE; i++)
				if(ram[i] < ram[i-1])
					sorted = 0;

			MBOX_PUT(resources_result, sorted);	
		}
		else
		{
			MBOX_PUT(resources_result, 0xffffffff);	
		}





	}
}