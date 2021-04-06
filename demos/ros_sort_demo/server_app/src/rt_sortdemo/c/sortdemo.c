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
		ROS_SUBSCRIBE_TAKE(resources_subdata, resources_sort_msg);
		printf("Received new data (len = %d, cap = %d)! \n", resources_sort_msg->sortdata.size, resources_sort_msg->sortdata.capacity);
		bubblesort(resources_sort_msg->sortdata.data, resources_sort_msg->sortdata.size);
		printf("Publish new data! \n");
		ROS_PUBLISH(resources_pubdata, resources_sort_msg  );
	}
}
