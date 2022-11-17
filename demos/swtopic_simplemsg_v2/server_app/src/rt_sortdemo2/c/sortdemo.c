#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048


#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048



void *rt_sortdemo2(void *data) {
	clock_t start, end;
	while (1) {
		//ROS_SUBSCRIBE_TAKE(resources_subdata, resources_sort_msg);
		start = clock();
		//printf("Received new data (len = %d, cap = %d)! \n", resources_sort_msg->data.size, resources_sort_msg->data.capacity);
		//sort_net(resources_sort_msg->data.data, resources_sort_msg->data.size);
		//printf("Publish new data! \n");
		end = clock();
		//ROS_PUBLISH(resources_pubdata, resources_sort_msg  );

		printf("%3.6f\n", (double)(end-start)/CLOCKS_PER_SEC);
	}
}