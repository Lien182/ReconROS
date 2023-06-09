#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#define DEFAULT_IMAGE_HEIGHT 1000
#define DEFAULT_IMAGE_WIDTH 1000



#define log(...) printf(__VA_ARGS__); fflush(stdout)



int main(int argc, char **argv) {

	printf("Start init \n");

	reconos_init();
	reconos_app_init();


	// gateway image

	rthreada_img_output->header.frame_id.data=malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    rthreada_img_output->encoding.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	                
    rthreada_img_output->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	

	gateway1_ressourcegroup_msg_out->header.frame_id.data=malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
    gateway1_ressourcegroup_msg_out->encoding.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	                
    gateway1_ressourcegroup_msg_out->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);	

	
	memset(rthreada_img_output->header.frame_id.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
	memset(rthreada_img_output->encoding.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
	memset(rthreada_img_output->data.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);

	memset(gateway1_ressourcegroup_msg_out->header.frame_id.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
	memset(gateway1_ressourcegroup_msg_out->encoding.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);
	memset(gateway1_ressourcegroup_msg_out->data.data, 0, DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH*3);

	reconos_thread_create_hwt_threada(0);
	reconos_thread_create_hwt_threadb(0);

	reconos_thread_create_hwt_gateway1(0);

	ros_subscriber_set_publishfilter(gateway1_ressourcegroup_sub, gateway1_ressourcegroup_pub->guid);

	while(1)
	{
		sleep(1);

		//ros_subscriber_message_take(gateway1_ressourcegroup_sub, gateway1_ressourcegroup_msg_in);
		//printf("Message arrived! \n");
		//printf("size = %d, data = %x \n", gateway1_ressourcegroup_msg_in->encoding.size, gateway1_ressourcegroup_msg_in->encoding.data);

		//uint32_t val = mbox_get(rthreada_debug_mbox);
		//printf("Message arrived! \n");

		// printf("gateway1_ressourcegroup_msg_out->height = %d \n", rthreada_img_output->height);
		// printf("gateway1_ressourcegroup_msg_out->width = %d \n", rthreada_img_output->width);

		// printf("gateway1_ressourcegroup_msg_out->encoding.data = %s \n", rthreada_img_output->encoding.data);
		// printf("gateway1_ressourcegroup_msg_out->encoding.size = %d \n", rthreada_img_output->encoding.size);
		// printf("gateway1_ressourcegroup_msg_out->encoding.capacity = %d \n", rthreada_img_output->encoding.capacity);
		// printf("gateway1_ressourcegroup_msg_out->header.frame_id.data = %s \n", rthreada_img_output->header.frame_id.data);
		// printf("gateway1_ressourcegroup_msg_out->header.frame_id.size = %d \n", rthreada_img_output->header.frame_id.size);
		// printf("gateway1_ressourcegroup_msg_out->header.frame_id.capacity = %d \n", rthreada_img_output->header.frame_id.capacity);
		
	} 

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}