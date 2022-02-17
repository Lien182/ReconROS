/*  Author   :   Shreejith S
 *  File     :   zycap_linux.c
 *  Project  :   ZyCAP User-level driver for Linux
 *  Dcpr.    :   Management of bitstreams and buffers.
 */

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include "zycap_linux.h"

#define DevCfg_BASE_ADDRESS 0xF8007000
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define BS_BASEADDR
#define FIFO_LEN (20*1024*1024)


static pthread_mutex_t __mutexPR = PTHREAD_MUTEX_INITIALIZER;;

int memCpy_DMA(int fd, char *bufferIn, unsigned long elems) 
{

	int byteMoved  = 0;
	
    if(elems < FIFO_LEN)
    {
	    byteMoved = write(fd,   (char *)bufferIn, elems );
	}

	return byteMoved;
}


int Zycap_Init(t_zycap * zycap)
{
    int data;
    int memfd;
    int fd;
    void *mapped_base, *mapped_dev_base;
    off_t dev_base = DevCfg_BASE_ADDRESS;
    zycap->memfd = -1;
    zycap->fd = 0;
    memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (memfd == -1) {
        printf("[Zycap] Can't open /dev/mem.\n");
        exit(0);
    }
    printf("[Zycap] /dev/mem opened.\n");
    mapped_base = mmap(0, MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,memfd,dev_base&~MAP_MASK);
    if (mapped_base == (void *) -1) {
        printf("[Zycap] Can't map the memory to user space.\n");
        return -1;
    }
    mapped_dev_base = mapped_base + (dev_base & MAP_MASK);
    *((volatile unsigned long *) (mapped_dev_base))=0x4200E07F;
    data = *((volatile unsigned long *) (mapped_dev_base));
    printf("[Zycap] DevCfg Control Reg: %0x\n",data);



	printf("[Zycap] Zycap Initialised \n");
 
    fd = open("/dev/zycap-vivado", O_SYNC | O_RDWR);  
    if(fd < 0)
    {
       printf("[Zycap] cannot get the file descriptor \n");
       return -1;
    }
    else 
    {
      printf("[Zycap] Zycap Opened \n");
    }
    zycap->memfd = memfd;
    zycap->fd = fd;
     
	return memfd;	
}
	

   
    
int Zycap_Write_Bitstream       (t_zycap * zycap, t_bitstream * bitstream)
{
	int bytesMoved;
    
    if(bitstream->data != 0 && bitstream->size != 0)
    {
        pthread_mutex_lock(&__mutexPR);
	    bytesMoved = memCpy_DMA(zycap->fd, bitstream->data, bitstream->size); 
        pthread_mutex_unlock(&__mutexPR);
    }
    else
    {
        return -1;
    }

	return bytesMoved;
}	




int Zycap_Deinit (t_zycap * zycap )
{
    close(zycap->fd);
	return 1;
}

int Zycap_Prefetch_Bitstream    ( char * bs_name, t_bitstream * bitstream)
{
    int bs_pres;
    int pres_first;
    int pres_last;
    int size;
	FILE *fd,*fp;   
	char fname[100];     
    
    strcpy (fname, bs_name);
    strcat (fname,".bin");
    
    fp = fopen(fname,"rb");
    
    if(!fp)
    {
        printf("Unable to open PR file\n");
        return -1;
    }
    
    //printf("Reading PR file from flash\n\r");
    fseek(fp,0,SEEK_END);
    size=ftell(fp);
    fseek(fp,0,SEEK_SET);

    if (bitstream->data == 0 || bitstream->size != 0)
    {
        bitstream->data = malloc(size); 
        if(bitstream->data == 0)
        {
            printf("[Zycap] bitstream prefetch %s: malloc failed \n", bs_name);
            return -1;
        }

    }
    else
    {
        printf("[Zycap] bitstream prefetch %s: has already an address: %x \n", bs_name, (uint32_t)bs_name);
    }
    int bytes_read = fread(bitstream->data,1,size,fp);
    bitstream->size = size;
    fclose(fp); 

    printf("[Zycap] bitstream prefetch %s: %d bytes read\n", bs_name, bytes_read);
    
    
	return 1;
}
