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
		ROS_SERVICESERVER_TAKE(resources_srv, resources_sort_srv_req);
		printf("Received new data (len = %d, cap = %d)! \n", resources_sort_srv_req->unsorted.size, resources_sort_srv_req->unsorted.capacity);
		bubblesort(resources_sort_srv_req->unsorted.data, resources_sort_srv_req->unsorted.size);
		printf("Publish new data! \n");
		memcpy(resources_sort_srv_res, resources_sort_srv_req, sizeof(resources_sort_srv_req_s));
		ROS_SERVICESERVER_SEND_RESPONSE(resources_srv, resources_sort_srv_res  );
		
	}
}
