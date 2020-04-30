#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BLOCK_SIZE 2048




#define log(...) printf(__VA_ARGS__); fflush(stdout)

void print_help() {
	printf("\n"
	       "ReconOS v4 sort demo application\n"
	       "--------------------------------\n"
	       "\n"
	       "Sorts a buffer full of data with a variable number of sw and hw threads.\n"
	       "\n"
	       "Usage:\n"
	       "    sort_demo <num_hw_threads> <num_sw_threads>\n"
	       "\n"
	       "    <num_hw_threads> - Number of hardware threads to create. The maximum number is\n"
	       "                       limited by the hardware design.\n"
	       "    <num_sw_threads> - Number of software threads to create.\n"
	       "\n"
	);
}

int cmp_uint32t(const void *a, const void *b) {
	return *(uint32_t *)a - *(uint32_t *)b;
}

void _merge(uint32_t *data, uint32_t *tmp,
           int l_count, int r_count) {
	int i;
	uint32_t *l = data, *r = data + l_count;
	int li = 0, ri = 0;

	for (i = 0; i < l_count; i++) {
		tmp[i] = l[i];
	}

	for (i = 0; i < l_count + r_count; i++) {
		if (ri >= r_count || (li < l_count && tmp[li] < r[ri])) {
			data[i] = tmp[li];
			li++;
		} else {
			data[i] = r[ri];
			ri++;
		}
	}
}

void merge(uint32_t *data, int data_count) {
	int bs, bi;
	uint32_t *tmp;

	tmp = (uint32_t *)malloc(data_count * sizeof(uint32_t));

	for (bs = BLOCK_SIZE; bs < data_count; bs += bs) {
		for (bi = 0; bi < data_count; bi += bs + bs) {
			if (bi + bs + bs > data_count) {
				_merge(data + bi, tmp, bs, data_count - bi - bs);
			} else {
				_merge(data + bi, tmp, bs, bs);
			}
		}
	}

	free(tmp);
}

int main(int argc, char **argv) {
	int i;
	int num_hwts, num_swts;
	int clk;

	if (argc != 3) {
		print_help();
		return 0;
	}

	num_hwts = atoi(argv[1]);
	num_swts = atoi(argv[2]);

	printf("Start init \n");

	reconos_init();
	reconos_app_init();

	// VERY; VERY UGLY
	memcpy(&resources_sort_msg_s, resources_sort_msg, sizeof(resources_sort_msg_s));
	resources_sort_msg = &resources_sort_msg_s;


	clk = reconos_clock_threads_set(100000);

	log("creating %d hw-threads(clk = %d):", num_hwts,clk);
	for (i = 0; i < num_hwts; i++) {
		log(" %d", i);
		reconos_thread_create_hwt_sortdemo(0);
	}
	log("\n");

	log("creating %d sw-thread:", num_swts);
	for (i = 0; i < num_swts; i++) {
		log(" %d", i);
		reconos_thread_create_swt_sortdemo(0,0);
	}
	log("\n");

	while(1)
	{
		sleep(1);
		
	} 

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}