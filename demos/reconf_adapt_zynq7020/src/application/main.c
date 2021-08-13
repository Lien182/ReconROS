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

#include "main.h"

#include "zycap_linux.h"


#define MAX_THREADS 5

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
  	rsobel_image_msg_out->height = IMAGE_HEIGHT;
  	rsobel_image_msg_out->width = IMAGE_WIDTH;
  	rsobel_image_msg_out->encoding.data = "bgr8";
	rsobel_image_msg_out->encoding.size = 4;  
	rsobel_image_msg_out->encoding.capacity = 5;  
  	rsobel_image_msg_out->is_bigendian = 0;
  	rsobel_image_msg_out->step = IMAGE_WIDTH*3;
  	rsobel_image_msg_out->data.data = malloc(IMAGE_HEIGHT*IMAGE_WIDTH*3);
	rsobel_image_msg_out->data.size = IMAGE_HEIGHT*IMAGE_WIDTH*3;
	rsobel_image_msg_out->data.capacity = IMAGE_HEIGHT*IMAGE_WIDTH*3;

}


void timespec_diff(struct timespec *start, struct timespec *stop,
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
	uint32_t bUseHW = 0;
	uint32_t nThreads = 0;
	
	reconos_init();
	reconos_app_init();

	uint32_t inverse_result;


	struct reconos_thread* threads[MAX_THREADS];

	
	init_zycap();
	
	init_msg();
	
	Prefetch_PR_Bitstream("sortdemo_0");
	Prefetch_PR_Bitstream("sortdemo_1");
	Prefetch_PR_Bitstream("sobel_0");
	Prefetch_PR_Bitstream("sobel_1");
	Prefetch_PR_Bitstream("mnist_0");
	Prefetch_PR_Bitstream("mnist_1");



	clock_gettime(CLOCK_MONOTONIC, &t_start);
	//Config_PR_Bitstream("sortdemo");
	clock_gettime(CLOCK_MONOTONIC, &t_end);
	timespec_diff(&t_start, &t_end, &t_res);
	printf("%3.6f;\n", (double)(t_res.tv_nsec)/1000000000);


	if(argc == 3)
	{
		if(strcmp(argv[1], "sw") == 0)
		{
			bUseHW = 0;
			printf("Starting Software Thread \n");
		}
		else
		{
			bUseHW = 1;
			printf("Starting Hardware Thread \n");
		}	
		nThreads = atoi(argv[2]);
		printf("Number of threads %d \n", nThreads);

		
	}
	else
	{
		printf("Usage: ./reconfadapt <sw/hw> <#threads>\n");
		return -1;
	}

	if(bUseHW == 1)
	{
		for(int i = 0; i < nThreads; i++)
		{
			//threads[i] = reconos_thread_create_hwt_sobel(rsobel_image_msg_out->data.data);
			threads[i] = reconos_thread_create_hwt_mnist(&rmnist_output_msg->data);
		}
		
	}
	else
	{
		for(int i = 0; i < nThreads; i++)
		{
			//threads[i] = reconos_thread_create_swt_sobel(0,0);
			threads[i] = reconos_thread_create_swt_mnist(0,0);
		}
		
	}
	


	uint32_t count = 0;

	uint32_t nMsgSobel = 0;
	uint32_t nMsgSort = 0;
	uint32_t nMsgMnist = 0;

	while(1)
	{
		
		usleep(200000);
		
		uint32_t ret = 0;

		ret |= rcl_subscription_get_unread_count(&rsobel_subdata->sub, &nMsgSobel);
		ret |= rcl_service_get_unread_requests(&rsort_srv->service, &nMsgSort);
		ret |= rcl_subscription_get_unread_count(&rmnist_subdata->sub, &nMsgMnist);
		
		
		printf("Unread requests: ret = %d \n", ret);
		printf("Sobel=%06d,Mnist=%06d,Sort=%06d \n", nMsgSobel ,nMsgMnist, nMsgSort);

	}

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}