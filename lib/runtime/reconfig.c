#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <poll.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/stat.h>
#include <fcntl.h>


int reconos_reconfigure_legacy(char * bitstream, unsigned int length, unsigned int partial)
{
	/* construct path of bitfile */

	int fd_partial = open("/sys/devices/soc0/amba/f8007000.devcfg/is_partial_bitstream", O_RDWR);
	if(fd_partial < 0){
		printf("Failed to open xdevcfg attribute 'is_partial_bitstream' when configuring\n");
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

	fd_partial = open("/dev/xdevcfg", O_WRONLY);
	if(fd_partial < 0){
		printf("Failed to open xdevcfg device when configuring\n");
		return -1;
	}
	printf("Opened xdevcfg. Configuring with %u bytes\n",length);
	write(fd_partial, bitstream, length);
	int fd_finish_flag = open("/sys/devices/soc0/amba/f8007000.devcfg/prog_done", O_RDWR);
	if(fd_finish_flag < 0){
		printf("Failed to open fd_finish_flag when configuring\n");
		return -1;
	}
	char finish_flag = '0';

	/* wait until reconfiguration is finished */
	while(finish_flag != '1'){
		read(fd_finish_flag,&finish_flag,1);
	}
	printf("Reconfiguration with bitfile finished\n");
	close(fd_partial);
	close(fd_finish_flag);

	return 0;

}
int reconos_reconfigure_legacy_fs(char * bitstream_path, unsigned int partial)
{
	char command[2048];

	snprintf(command, sizeof(command), "cat %s > /dev/xdevcfg", bitstream_path);
	system(command);

	return 0;

}


int fpga_state()
{
	FILE *fptr;
	char buf[10];
    const char *state = "operating";
	fptr = fopen("/sys/class/fpga_manager/fpga0/state", "r");
    if (fptr) 
	{
		fgets(buf, 10, fptr);
        printf("Stream read: %s \n", buf);
		fclose(fptr);
		if (strcmp(buf, state) == 0)
			return 0;
		else
			return 1;
	}
	
	return 1;	
}



int gettime(struct timeval  t0, struct timeval t1)
{
        return ((t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec -t0.tv_usec) / 1000.0f);
}


int reconos_reconfigure_fpgamgr(char * bitstream, unsigned int length, unsigned int partial)
{
    struct timeval t1, t0;
    double time;


    int fd_partial = open("/sys/class/fpga_manager/fpga0/flags", O_WRONLY);
	if(fd_partial < 0){
		printf("Failed to open/sys/class/fpga_manager/fpga0/flags\n");
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

    gettimeofday(&t0, NULL);
    fd_partial = open("/sys/class/fpga_manager/fpga0/firmware", O_WRONLY);
	if(fd_partial < 0){
		printf("Failed to open /sys/class/fpga_manager/fpga0/firmware device when configuring\n");
		return -1;
	}
	printf("Opened xdevcfg. Configuring with %u bytes\n",length);
	write(fd_partial, bitstream, length);
	
    close(fd_partial);
	
    gettimeofday(&t1, NULL);
    time = gettime(t0, t1);
    if (!fpga_state()) {
       printf("Time taken to load BIN is %f Milli Seconds\n\r", time);
        printf("BIN FILE loaded through FPGA manager successfully\n\r");
    } else {
        printf("BIN FILE loading through FPGA manager failed\n\r");
    }


    return 0;
}

int reconos_reconfigure_fpgamgr_fs(char * bitstream_path, unsigned int partial)
{
	char command[2048];
	char *tmp, *tmp1, *token;
	struct timeval t1, t0;
    double time;

	snprintf(command, sizeof(command), "echo %x > /sys/class/fpga_manager/fpga0/flags", partial);
	system(command);
	printf("DEBUG=%s\n", command);

	tmp = strdup(bitstream_path);
	while((token = strsep(&tmp, "/"))) {
		tmp1 = token;
	}
	snprintf(command, sizeof(command), "echo %s > /sys/class/fpga_manager/fpga0/firmware", tmp1);
	printf("DEBUG=%s\n", command);
	gettimeofday(&t0, NULL);
	system(command);
	gettimeofday(&t1, NULL);
	time = gettime(t0, t1);
	if (!fpga_state()) {
		printf("Time taken to load BIN is %f Milli Seconds\n\r", time);
		printf("BIN FILE loaded through FPGA manager successfully\n\r");
	} else {
		printf("BIN FILE loading through FPGA manager failed\n\r");
	}
	//snprintf(command, sizeof(command), "rm /lib/firmware/%s", tmp1);
	//system(command);
}
