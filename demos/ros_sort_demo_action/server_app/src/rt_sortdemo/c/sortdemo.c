#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048



void sort_net(uint32_t * ram, uint32_t size) {
	unsigned int i,  stage;
	uint32_t tmp;

	for(stage = 1; stage <= size; stage++){
		
		for(i = 0; i < size - 1; i += 2){
			if (ram[i] > ram[i + 1]) {
				tmp = ram[i];
				ram[i] = ram[i + 1];
				ram[i + 1] = tmp;
			}
		}

		for(i = 1; i < size - 1; i += 2){
			if (ram[i] > ram[i + 1]) {
				tmp = ram[i];
				ram[i] = ram[i + 1];
				ram[i + 1] = tmp;
			}
		}
		if((stage % 256) == 0)
		{
			resources_sort_action_feedback->goal_id = resources_sort_action_goal_req->goal_id;
			resources_sort_action_feedback->feedback.percent = (float)stage /(float)size * 100.0f;
			ROS_ACTIONSERVER_FEEDBACK(resources_actionsrv,resources_sort_action_feedback);
			printf("Feedback sent \n");
			usleep(100000);
		}
			

	}
}


void *rt_sortdemo(void *data) {
	
	while (1) {
		
		printf("Wait for new data! \n");
		ROS_ACTIONSERVER_GOAL_TAKE(resources_actionsrv, resources_sort_action_goal_req);
		printf("Received new data (len = %d, cap = %d)! \n", resources_sort_action_goal_req->goal.unsorted.size, resources_sort_action_goal_req->goal.unsorted.capacity);
		ROS_ACTIONSERVER_GOAL_DECIDE(resources_actionsrv, ROS_ACTIONSERVER_GOAL_ACCEPT);
		
		printf("Decision done, now waiting for the result request. \n");
		ROS_ACTIONSERVER_RESULT_TAKE(resources_actionsrv);
		memcpy(&resources_sort_action_result_res->result.sorted, &resources_sort_action_goal_req->goal.unsorted, sizeof(resources_sort_action_goal_req->goal.unsorted));
		sort_net(resources_sort_action_goal_req->goal.unsorted.data, resources_sort_action_goal_req->goal.unsorted.size);
		printf("Publish new data! \n");
		
		//Sort in place, boths messages refer to the same data location
		ROS_ACTIONSERVER_RESULT_SEND(resources_actionsrv, resources_sort_action_result_res  );
		
	}
}
