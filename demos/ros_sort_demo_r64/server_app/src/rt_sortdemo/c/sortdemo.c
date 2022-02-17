#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048


#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048


void sort_net(uint32_t *data, int data_count) {
	unsigned int i, k, stage;
	uint32_t tmp;

	for(stage = 1; stage <= data_count; stage++){
		k = (stage % 2 == 1) ? 0 : 1;
		for(i = k; i < data_count - 1; i += 2){
			if (data[i] > data[i + 1]) {
				tmp = data[i];
				data[i] = data[i + 1];
				data[i + 1] = tmp;
			}
		}
	}
}

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
	clock_t start, end;
	while (1) {
		ROS_SUBSCRIBE_TAKE(resources_subdata, resources_sort_msg);
		start = clock();
		//printf("Received new data (len = %d, cap = %d)! \n", resources_sort_msg->data.size, resources_sort_msg->data.capacity);
		sort_net(resources_sort_msg->data.data, resources_sort_msg->data.size);
		//printf("Publish new data! \n");
		end = clock();
		ROS_PUBLISH(resources_pubdata, resources_sort_msg  );

		printf("%3.6f\n", (double)(end-start)/CLOCKS_PER_SEC);
	}
}