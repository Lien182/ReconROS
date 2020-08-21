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
	uint32 addr, initdata;	
	uint32 pMessage;
	uint32 payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		pMessage = ROS_ACTIONSERVER_GOAL_TAKE(resources_actionsrv, resources_sort_action_goal_req);
		addr = OFFSETOF(sorter_msgs__action__Sort_SendGoal_Request, goal.unsorted.data) + pMessage;

		MEM_READ(addr, payload_addr, 4);					//Get the address of the data
		MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);
		ROS_ACTIONSERVER_GOAL_DECIDE(resources_actionsrv, ROS_ACTION_SERVER_GOAL_ACCEPT);
		ROS_ACTIONSERVER_RESULT_TAKE(resources_actionsrv);
		sort_bubble(ram);

		pMessage = ROS_MESSAGE_ARRAY_SET_SIZE(resources_sort_action_result_res,  OFFSETOF(sorter_msgs__action__Sort_GetResult_Response, result.sorted.data), 4,   BLOCK_SIZE);
		MEM_WRITE(ram, pMessage, BLOCK_SIZE * 4);
		
		ROS_ACTIONSERVER_RESULT_SEND(resources_actionsrv, resources_sort_action_result_res  );
	}
}


/*
		printf("Wait for new data! \n");
		ROS_ACTIONSERVER_GOAL_TAKE(resources_actionsrv, resources_sort_action_goal_req);
		printf("Received new data (len = %d, cap = %d)! \n", resources_sort_action_goal_req->goal.unsorted.size, resources_sort_action_goal_req->goal.unsorted.capacity);
		ROS_ACTIONSERVER_GOAL_DECIDE(resources_actionsrv, ROS_ACTION_SERVER_GOAL_ACCEPT);
		
		printf("Decision done, now waiting for the result request. \n");
		ROS_ACTIONSERVER_RESULT_TAKE(resources_actionsrv);
		sort_net(resources_sort_action_goal_req->goal.unsorted.data, resources_sort_action_goal_req->goal.unsorted.size);
		printf("Publish new data! \n");
		
		//Sort in place, boths messages refer to the same data location
		memcpy(&resources_sort_action_result_res->result.sorted, &resources_sort_action_goal_req->goal.unsorted, sizeof(resources_sort_action_goal_req->goal.unsorted));
		ROS_ACTIONSERVER_RESULT_SEND(resources_actionsrv, resources_sort_action_result_res  );
*/