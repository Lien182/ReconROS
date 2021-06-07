#define  _GNU_SOURCE

#define RECONOS_DEBUG

#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"
#include "timer.h"

#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>


void init_msg(void)
{
	//std_msgs__msg__Header header;
  	rsobel_image_msg_out->height = 480;
  	rsobel_image_msg_out->width = 640;
  	rsobel_image_msg_out->encoding.data = "bgr8";
	rsobel_image_msg_out->encoding.size = 4;  
	rsobel_image_msg_out->encoding.capacity = 5;  
  	rsobel_image_msg_out->is_bigendian = 0;
  	rsobel_image_msg_out->step = 1920;
  	rsobel_image_msg_out->data.data = malloc(480*640*3);
	rsobel_image_msg_out->data.size = 480*640*3;
	rsobel_image_msg_out->data.capacity = 480*640*3;

}



int main(int argc, char **argv) {

	reconos_init();
	reconos_app_init();

	uint32_t inverse_result;


	struct reconos_thread* threads[10];


	init_msg();

	threads[0] = reconos_thread_pr_create_hwt_sortdemo(0, "bitstreams");

	threads[1] = reconos_thread_pr_create_hwt_inverse(&inverse_result, "bitstreams");
	sleep(5);
	printf("Suspend block 0 \n");
	reconos_thread_suspend_block(threads[0]);
	printf("Start new thread \n");
	threads[2] = reconos_thread_pr_create_hwt_blur(rsobel_image_msg_out->data.data, "bitstreams");

	while(1)
	{
		sleep(10);
	}

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}