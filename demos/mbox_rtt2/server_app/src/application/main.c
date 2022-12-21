#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BLOCK_SIZE 2048

#define DEFAULT_IMAGE_HEIGHT 100
#define DEFAULT_IMAGE_WIDTH 100

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



#define log(...) printf(__VA_ARGS__); fflush(stdout)

void print_help() {
	printf("\n"
	       "ReconOS v4 hwtopic demo application\n"
	       "--------------------------------\n"
	       "\n"
	       "Subscribes to a topic, sends it via a hwtopic and publishes to a normal ROS topic .\n"
	       "\n"
	       "\n"
	);
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

	log("creating %d hw-threads(clk = %d):", num_hwts,clk);

	reconos_thread_create_hwt_athreada(0);
	reconos_thread_create_hwt_athreadb(0);
	reconos_thread_create_hwt_athreadc(0);
	uint64_t stuff = 0;

	struct timespec t_start, t_end, t_res;

	uint16_t it_counter = 0;
	while(1)
	{
		sleep(1);
		clock_gettime(CLOCK_MONOTONIC, &t_start);
		for(uint16_t i = 0; i < 1000; i++)
		{
			mbox_put(rthreadb_start_mbox, stuff); // rthreadb_start_mbox
			mbox_get(rthreadb_finish_mbox); // rthreada_finish_mbox
		}
		
		clock_gettime(CLOCK_MONOTONIC, &t_end);
		timespec_diff(&t_start, &t_end, &t_res);
		printf("[---reconos-dt-main] (mbox time * 1000) : %3.6f \n", (double)(t_res.tv_nsec)/1000000000);
		it_counter++;
		if(it_counter == 1000)
		{
			break;
		}
	} 

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}