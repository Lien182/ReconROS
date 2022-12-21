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
	

		// output image

	rthreadc_img_output->header.stamp.sec = 0;
	rthreadc_img_output->header.stamp.nanosec = 0;
	rthreadc_img_output->header.frame_id.size=7;
	rthreadc_img_output->header.frame_id.capacity=8;
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

	

	while(1)
	{
		sleep(1);
		
	} 

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}