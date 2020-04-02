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
	uint32_t ret, len;

	uint8_t * sort_data;

	while (1) {
		MBOX_TRYGET(resources_address, ret);

		if (ret == 0xffffffff) {
			THREAD_EXIT();
		}
		printf("Wait for new data! \n");
		ROS_SUBSCRIBE_TAKE(resources_subdata, sort_data, len);
		printf("Received new data (len = %d)! \n", len);
		bubblesort((uint32_t*)sort_data, len >> 2);
		printf("Publish new data! \n");
		ROS_PUBLISH(resources_pubdata, sort_data, len  );
	}
}
