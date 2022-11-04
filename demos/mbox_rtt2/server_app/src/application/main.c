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

	//struct reconos_resource res[2];
	uint8_t img_1[DEFAULT_IMAGE_HEIGHT * DEFAULT_IMAGE_WIDTH * 3];
	uint8_t img_2[DEFAULT_IMAGE_HEIGHT * DEFAULT_IMAGE_WIDTH * 3];

	// output image
	rthreada_img_output->header.stamp.sec = 0;

	log("a->header.stamp.sec: %p\n", (void*)&rthreada_img_output->header.stamp.sec);

	rthreada_img_output->header.stamp.nanosec = 0;
	log("a->header.stamp.nanosec: %p\n", (void*)&rthreada_img_output->header.stamp.nanosec);

	rthreada_img_output->header.frame_id.size=7;

	log("a->header.frame_id.size: %p\n", (void*)&rthreada_img_output->header.frame_id.size);

	rthreada_img_output->header.frame_id.capacity=8;

	log("a->header.frame_id.capacity: %p\n", (void*)&rthreada_img_output->header.frame_id.capacity);

	rthreada_img_output->header.frame_id.data="______\n";


	rthreada_img_output->height = DEFAULT_IMAGE_HEIGHT;	        
        
    rthreada_img_output->width = DEFAULT_IMAGE_WIDTH;	        
        
    rthreada_img_output->encoding.data = "bgr8";	        
        
    rthreada_img_output->encoding.size = 4;	        
        
    rthreada_img_output->encoding.capacity = 8;	        
        
    rthreada_img_output->is_bigendian = 0;	        
        
    rthreada_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreada_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);

	log("a->header.data.data[0]: %p\n", rthreada_img_output->data.data);
	log("a->header.data.data[1]: %p\n", rthreada_img_output->data.data+1);
	log("a->header.data.data[-1]: %p\n", (void*)&rthreada_img_output->data.data[DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3-1]);	        
        
    rthreada_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreada_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

	///////
	rthreadb_img_output_hw->header.stamp.sec = 0;
	rthreadb_img_output_hw->header.stamp.nanosec = 0;
	rthreadb_img_output_hw->header.frame_id.size=7;
	rthreadb_img_output_hw->header.frame_id.capacity=8;
	rthreadb_img_output_hw->header.frame_id.data="______\n";
	rthreadb_img_output_hw->height = DEFAULT_IMAGE_HEIGHT;	        
    rthreadb_img_output_hw->width = DEFAULT_IMAGE_WIDTH;	         
    rthreadb_img_output_hw->encoding.data = "bgr8";	        
    rthreadb_img_output_hw->encoding.size = 4;	          
    rthreadb_img_output_hw->encoding.capacity = 8;	        
    rthreadb_img_output_hw->is_bigendian = 0;	            
    rthreadb_img_output_hw->step = DEFAULT_IMAGE_WIDTH*3;	          
    rthreadb_img_output_hw->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);  
    rthreadb_img_output_hw->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	           
    rthreadb_img_output_hw->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;


	
	// input image
	/*
	rthreadb_img_input->header.frame_id.size = 0;

	rthreadb_img_input->header.frame_id.capacity = 8;

	rthreadb_img_input->header.frame_id.data = "_______\n";

	rthreadb_img_input->height = DEFAULT_IMAGE_HEIGHT;	        
        
    rthreadb_img_input->width = DEFAULT_IMAGE_WIDTH;	        
        
    rthreadb_img_input->encoding.data = "bgr8___\n";	        
        
    rthreadb_img_input->encoding.size = 8;	        
        
    rthreadb_img_input->encoding.capacity = 8;	        
        
    rthreadb_img_input->is_bigendian = 0;	        
        
    rthreadb_img_input->step = DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreadb_img_input->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	        
        
    rthreadb_img_input->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreadb_img_input->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	
	*/

		// output image

	rthreadc_img_output->header.stamp.sec = 0;

	log("c->header.stamp.sec: %p\n", (void*)&rthreadc_img_output->header.stamp.sec);

	rthreadc_img_output->header.stamp.nanosec = 0;

	log("c->header.stamp.nanosec: %p\n", (void*)&rthreadc_img_output->header.stamp.nanosec);
	
	rthreadc_img_output->header.frame_id.size=7;

	log("c->header.frame_id.size: %p\n", (void*)&rthreadc_img_output->header.frame_id.size);

	rthreadc_img_output->header.frame_id.capacity=8;

	log("c->header.frame_id.capacity: %p\n", (void*)&rthreadc_img_output->header.frame_id.capacity);

	rthreadc_img_output->header.frame_id.data="______\n";


	rthreadc_img_output->height = DEFAULT_IMAGE_HEIGHT;	        
        
    rthreadc_img_output->width = DEFAULT_IMAGE_WIDTH;	        
        
    rthreadc_img_output->encoding.data = "bgr8";	        
        
    rthreadc_img_output->encoding.size = 4;	        
        
    rthreadc_img_output->encoding.capacity = 8;	        
        
    rthreadc_img_output->is_bigendian = 0;	        
        
    rthreadc_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreadc_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	        
        
    rthreadc_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
        
    rthreadc_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

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