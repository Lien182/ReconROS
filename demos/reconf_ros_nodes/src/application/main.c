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


#include "zycap_linux.h"

int reconfigure(char* filename,unsigned int partial){
	/* construct path of bitfile */

	FILE *bitfile;
	unsigned int size;
	char *bitstream;

	bitfile = fopen(filename, "rb");
	if(!bitfile){
		printf("Error opening bitfile %s\n",filename);
		return -1;
	}

	fseek(bitfile,0L,SEEK_END);
	size=ftell(bitfile);
	rewind(bitfile);

	bitstream = (char *)malloc(size*sizeof(char));
	if(!bitstream){
		printf("Error allocating memory for bitstream %s\n",filename);
		return -1;
	}
	fread(bitstream, sizeof(char), size, bitfile);
	fclose(bitfile);

	int fd_partial = open("/sys/devices/soc0/amba/f8007000.devcfg/is_partial_bitstream", O_RDWR);
	if(fd_partial < 0){
		printf("Failed to open xdevcfg attribute 'is_partial_bitstream' when configuring %s\n",filename);
		return -1;
	}

	char partial_flag[2];
	if(!partial) {
		strcpy(partial_flag,"0");
	}
	else {
		strcpy(partial_flag,"1");
	}
	write(fd_partial, partial_flag, 2);
	close(fd_partial);

	fd_partial = open("/dev/xdevcfg", O_RDWR);
	if(fd_partial < 0){
		printf("Failed to open xdevcfg device when configuring %s\n",filename);
		return -1;
	}
	//log("Opened xdevcfg. Configuring with %u bytes\n",size);
	write(fd_partial, bitstream, size);
	int fd_finish_flag = open("/sys/devices/soc0/amba/f8007000.devcfg/prog_done", O_RDWR);
	char finish_flag = '0';

	/* wait until reconfiguration is finished */
	while(finish_flag != '1'){
		read(fd_finish_flag,&finish_flag,1);
	}
	//log("Reconfiguration with bitfile %s finished\n",filename);
	close(fd_partial);
	close(fd_finish_flag);

	return 0;

}



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

	struct timespec t_start, t_end, t_res;
	
	reconos_init();
	reconos_app_init();

	uint32_t inverse_result;


	struct reconos_thread* threads[10];

	init_msg();

	Prefetch_PR_Bitstream("sortdemo");

	clock_gettime(CLOCK_MONOTONIC, &t_start);
	Config_PR_Bitstream("sortdemo");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("%3.6f;\n", (double)(t_res.tv_nsec)/1000000000);




	//threads[0] = reconos_thread_pr_create_hwt_sortdemo(0, "bitstreams");

	//threads[1] = reconos_thread_pr_create_hwt_inverse(&inverse_result, "bitstreams");
	sleep(5);
	printf("Suspend block 0 \n");
	reconos_thread_suspend_block(threads[0]);
	printf("Start new thread \n");
	//threads[2] = reconos_thread_pr_create_hwt_blur(rsobel_image_msg_out->data.data, "bitstreams");

	while(1)
	{
		sleep(10);
	}

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}