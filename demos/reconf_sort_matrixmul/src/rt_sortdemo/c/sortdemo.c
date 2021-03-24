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
	uint32_t ret;
	int flag;

	while (1) {
		flag = 0;
		while(!flag){
			flag = MBOX_TRYGET(resources_address,ret);
		}

		if (ret == 0xffffffff) {
			THREAD_EXIT();
		}

		bubblesort((uint32_t *)ret, BLOCK_SIZE);
		MBOX_PUT(resources_acknowledge, ret);
	}
}
