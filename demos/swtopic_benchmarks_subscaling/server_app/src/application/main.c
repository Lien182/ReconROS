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
	       "Spawns one ReconROS Topic publisher in hw and multiple subscribers .\n"
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

	// thread a 
	// output image
	rthreada_img_output->header.stamp.sec = 0;
	rthreada_img_output->header.stamp.nanosec = 0;
	rthreada_img_output->header.frame_id.size=7;
	rthreada_img_output->header.frame_id.capacity=8;
	rthreada_img_output->header.frame_id.data="______\n";
	rthreada_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada_img_output->encoding.data = "bgr8";	        
    rthreada_img_output->encoding.size = 4;	        
    rthreada_img_output->encoding.capacity = 8;	        
    rthreada_img_output->is_bigendian = 0;	        
    rthreada_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

	// a2
	rthreada2_img_output->header.stamp.sec = 0;
	rthreada2_img_output->header.stamp.nanosec = 0;
	rthreada2_img_output->header.frame_id.size=7;
	rthreada2_img_output->header.frame_id.capacity=8;
	rthreada2_img_output->header.frame_id.data="______\n";
	rthreada2_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada2_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada2_img_output->encoding.data = "bgr8";	        
    rthreada2_img_output->encoding.size = 4;	        
    rthreada2_img_output->encoding.capacity = 8;	        
    rthreada2_img_output->is_bigendian = 0;	        
    rthreada2_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada2_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada2_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada2_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;


	// a3
	rthreada3_img_output->header.stamp.sec = 0;
	rthreada3_img_output->header.stamp.nanosec = 0;
	rthreada3_img_output->header.frame_id.size=7;
	rthreada3_img_output->header.frame_id.capacity=8;
	rthreada3_img_output->header.frame_id.data="______\n";
	rthreada3_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada3_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada3_img_output->encoding.data = "bgr8";	        
    rthreada3_img_output->encoding.size = 4;	        
    rthreada3_img_output->encoding.capacity = 8;	        
    rthreada3_img_output->is_bigendian = 0;	        
    rthreada3_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada3_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada3_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada3_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

/*
	// a4
	rthreada4_img_output->header.stamp.sec = 0;
	rthreada4_img_output->header.stamp.nanosec = 0;
	rthreada4_img_output->header.frame_id.size=7;
	rthreada4_img_output->header.frame_id.capacity=8;
	rthreada4_img_output->header.frame_id.data="______\n";
	rthreada4_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada4_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada4_img_output->encoding.data = "bgr8";	        
    rthreada4_img_output->encoding.size = 4;	        
    rthreada4_img_output->encoding.capacity = 8;	        
    rthreada4_img_output->is_bigendian = 0;	        
    rthreada4_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada4_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada4_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada4_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

	// a5
	rthreada5_img_output->header.stamp.sec = 0;
	rthreada5_img_output->header.stamp.nanosec = 0;
	rthreada5_img_output->header.frame_id.size=7;
	rthreada5_img_output->header.frame_id.capacity=8;
	rthreada5_img_output->header.frame_id.data="______\n";
	rthreada5_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada5_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada5_img_output->encoding.data = "bgr8";	        
    rthreada5_img_output->encoding.size = 4;	        
    rthreada5_img_output->encoding.capacity = 8;	        
    rthreada5_img_output->is_bigendian = 0;	        
    rthreada5_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada5_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada5_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada5_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;

	// a6
	rthreada6_img_output->header.stamp.sec = 0;
	rthreada6_img_output->header.stamp.nanosec = 0;
	rthreada6_img_output->header.frame_id.size=7;
	rthreada6_img_output->header.frame_id.capacity=8;
	rthreada6_img_output->header.frame_id.data="______\n";
	rthreada6_img_output->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada6_img_output->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada6_img_output->encoding.data = "bgr8";	        
    rthreada6_img_output->encoding.size = 4;	        
    rthreada6_img_output->encoding.capacity = 8;	        
    rthreada6_img_output->is_bigendian = 0;	        
    rthreada6_img_output->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada6_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada6_img_output->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada6_img_output->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;
*/
	// input image
	/*
	rthreada_img_input_hw->header.stamp.sec = 0;
	rthreada_img_input_hw->header.stamp.nanosec = 0;
	rthreada_img_input_hw->header.frame_id.size=7;
	rthreada_img_input_hw->header.frame_id.capacity=8;
	rthreada_img_input_hw->header.frame_id.data="______\n";
	rthreada_img_input_hw->height = DEFAULT_IMAGE_HEIGHT;	         
    rthreada_img_input_hw->width = DEFAULT_IMAGE_WIDTH;	          
    rthreada_img_input_hw->encoding.data = "bgr8";	        
    rthreada_img_input_hw->encoding.size = 4;	        
    rthreada_img_input_hw->encoding.capacity = 8;	        
    rthreada_img_input_hw->is_bigendian = 0;	        
    rthreada_img_input_hw->step = DEFAULT_IMAGE_WIDTH*3;	        
    rthreada_img_input_hw->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada_img_input_hw->data.size = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;	        
    rthreada_img_input_hw->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3;
	*/

	// thread b
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

	reconos_thread_create_hwt_athreada(0);
	reconos_thread_create_hwt_athreadb(0);
	reconos_thread_create_hwt_athreada2(0);
	reconos_thread_create_hwt_athreada3(0);
	/*
	reconos_thread_create_hwt_athreada4(0);
	reconos_thread_create_hwt_athreada5(0);
	reconos_thread_create_hwt_athreada6(0);
	*/

	uint64_t stuff = 0; //must remain 0

	struct timespec t_start, t_end, t_res;

	uint16_t it_counter = 0;
	while(1)
	{
		sleep(0.05);
		clock_gettime(CLOCK_MONOTONIC, &t_start);
		mbox_put(rthreadb_start_mbox, stuff); // rthreadb_start_mbox
		mbox_get(rthreada_finish_mbox); // rthreada_finish_mbox
		mbox_get(rthreada2_finish_mbox);
		mbox_get(rthreada3_finish_mbox);
		/*
		mbox_get(rthreada4_finish_mbox);
		mbox_get(rthreada5_finish_mbox);
		mbox_get(rthreada6_finish_mbox);
		*/
		clock_gettime(CLOCK_MONOTONIC, &t_end);
		timespec_diff(&t_start, &t_end, &t_res);
		printf("iteration counter: %d\n", it_counter);
		printf("[reconos-main] (swtopic time) : %3.6f \n", (double)(t_res.tv_nsec)/1000000000);
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