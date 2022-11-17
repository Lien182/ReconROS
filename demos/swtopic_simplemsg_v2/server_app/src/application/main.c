#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"

#include <std_msgs/msg/u_int64_multi_array__struct.h>

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

static void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return;
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


	clk = reconos_clock_threads_set(100000);

	printf("pre-init\n");
	reconos_thread_create_hwt_sortdemo(0);
	printf("after init 1\n");
	reconos_thread_create_hwt_sortdemo2(0);
	printf("after init 2\n");

	struct timespec t_start, t_end, t_res;
	resources_sort_msg->data.data = malloc(BLOCK_SIZE * 4);
	while(1)
	{
		clock_gettime(CLOCK_MONOTONIC, &t_start);
		mbox_put(resources_start_mbox, 0); // rthreadb_start_mbox
		printf("after starting measuring\n");
		mbox_get(resources2_finish_mbox); // rthreada_finish_mbox
		clock_gettime(CLOCK_MONOTONIC, &t_end);
		timespec_diff(&t_start, &t_end, &t_res);
		printf("[-------------------reconos-dt-main] (swtopic time) : %3.6f \n", (double)(t_res.tv_nsec)/1000000000);
		sleep(0.5);
		
	} 

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}