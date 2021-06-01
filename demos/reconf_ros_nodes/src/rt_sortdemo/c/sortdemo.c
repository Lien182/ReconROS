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
		ROS_SERVICESERVER_TAKE(rsort_srv, rsort_sort_srv_req);
		printf("Received new data (len = %d, cap = %d)! \n", rsort_sort_srv_req->unsorted.size, rsort_sort_srv_req->unsorted.capacity);
		bubblesort(rsort_sort_srv_req->unsorted.data, rsort_sort_srv_req->unsorted.size);
		printf("Publish new data! \n");
		memcpy(rsort_sort_srv_res, rsort_sort_srv_req, sizeof(rsort_sort_srv_req_s));
		ROS_SERVICESERVER_SEND_RESPONSE(rsort_srv, rsort_sort_srv_res  );
		
	}
}
