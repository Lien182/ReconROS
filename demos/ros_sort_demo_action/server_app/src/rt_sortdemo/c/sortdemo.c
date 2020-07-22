#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048



void bubblesort(uint32_t *data, int data_count) {
	int i;
	uint32_t tmp;
	int s, n, newn;

	s = 1;
	n = data_count - 1;
	newn = n;

	while (s) {
		s = 0;
		for (i = 0; i < n; i++) {
			if (data[i] > data[i + 1]) {
				tmp = data[i];
				data[i] = data[i + 1];
				data[i + 1] = tmp;
				newn = i;
				s = 1;
			}
		}

		n = newn;
	}
}

void *rt_sortdemo(void *data) {
	
	while (1) {
		
		printf("Wait for new data! \n");
		ROS_ACTIONSERVER_GOAL_TAKE(resources_actionsrv, resources_sort_action_goal_req);
		printf("Received new data (len = %d, cap = %d)! \n", resources_sort_action_goal_req->unsorted.size, resources_sort_action_goal_req->unsorted.capacity);
		ROS_ACTIONSERVER_GOAL_DECIDE(resources_actionsrv, ROS_ACTION_SERVER_ACCEPT_GOAL);

		ROS_ACTIONSERVER_RESULT_TAKE(resources_actionsrv);
		bubblesort(resources_sort_action_goal_req->unsorted.data, resources_sort_action_goal_req->unsorted.size);
		printf("Publish new data! \n");
		#warning TODO
		memcpy(resources_sort_action_result_res, resources_sort_action_goal_req, sizeof(resources_sort_action_result_res_s));
		ROS_ACTIONSERVER_RESULT_SEND(resources_actionsrv, resources_sort_action_result_res  );
		
	}
}
