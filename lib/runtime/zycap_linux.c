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
#include <sys/mman.h>
#include "zycap_linux.h"

#define DevCfg_BASE_ADDRESS 0xF8007000
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define BS_BASEADDR
#define MAX_BS_NUM 5


static int check_bs_present(bs_info *bs_list,char *bs_name);
static int find_first_bs(bs_info *bs_list);
static int find_last_bs(bs_info *bs_list);
static void print_schedule(bs_info *bs_list);
bs_info *bs_list;
buffertype *bufferIn;


unsigned long gettime(){
	struct timeval temp;

	gettimeofday(&temp, NULL);

	return temp.tv_sec * 1000 * 1000 + temp.tv_usec;
}

static int check_bs_present(bs_info *bs_list,char *bs_name)
{
    int i;
    for(i=0;i<MAX_BS_NUM;i++)
    {
        if(strcmp(bs_list[i].name,bs_name) == 0)
            return i;
    }
    return -1;
}

bs_info * init_bs_list(int num_bs)
{
    int i;
    bs_info *bs_list;
    bs_list = (bs_info *)malloc(num_bs*sizeof(bs_info));
    bufferIn = (buffertype *)malloc(num_bs*sizeof(buffertype));
    for(i=0;i<num_bs;i++)
    {
        strcpy(bs_list[i].name,"Dummy");
        bs_list[i].size = -1;
        if(i==0)
            bs_list[i].prev = NULL;
        else
            bs_list[i].prev = &bs_list[i-1];
        if(i==num_bs-1)
            bs_list[i].next = NULL;
        else
            bs_list[i].next = &bs_list[i+1];
       // bs_list[i].addr = (char *) malloc(bitsize+1);
       bs_list[i].addr = (int)bufferIn[i].bitbuffer;
    }
    return bs_list;
}


int find_first_bs(bs_info *bs_list)
{
    int i;
    for(i=0;i<MAX_BS_NUM;i++)
    {
        if(bs_list[i].prev == NULL)
            return i;
    }
    return -1;
}

int find_last_bs(bs_info *bs_list)
{
    int i;
    for(i=0;i<MAX_BS_NUM;i++)
    {
        if(bs_list[i].next== NULL)
            return i;
    }
    return -1;
}


void print_schedule(bs_info *bs_list)
{
    bs_info *current_bs;
    int i;
    for(i=0;i<MAX_BS_NUM;i++)
    {
        current_bs = &bs_list[i];
        printf("BS Num : %d BS Name : %s  BS Prev %s BS Next %s BS Addr %x \n",i,current_bs->name,current_bs->prev,current_bs->next, current_bs->addr);
    }
}



unsigned long memCpy_DMA(FILE *fd, char *bufferIn, unsigned long elems) 

{
    #define FIFO_LEN 4*1024*1024 
	unsigned long byteMoved  = 0;
	unsigned long byteToMove = elems;
	int i;

	while(byteMoved < elems){
		byteToMove = elems - byteMoved > FIFO_LEN ? FIFO_LEN : elems - byteMoved;
		fwrite((char *)bufferIn, byteToMove, 1, fd);
		byteMoved += byteToMove;
	}

	close(fd);

	return byteMoved;
}



int init_zycap()
{
    int data;
    int memfd;
    FILE *fd;
    void *mapped_base, *mapped_dev_base;
    off_t dev_base = DevCfg_BASE_ADDRESS;
    glbs.memfd = -1;
    glbs.fd = NULL;
    memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (memfd == -1) {
        printf("Can't open /dev/mem.\n");
        exit(0);
    }
    printf("/dev/mem opened.\n");
    mapped_base = mmap(0, MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,memfd,dev_base&~MAP_MASK);
    if (mapped_base == (void *) -1) {
        printf("Can't map the memory to user space.\n");
        return -1;
    }
    mapped_dev_base = mapped_base + (dev_base & MAP_MASK);
    *((volatile unsigned long *) (mapped_dev_base))=0x4200E07F;
    data = *((volatile unsigned long *) (mapped_dev_base));
    printf("DevCfg Control Reg: %0x\n",data);
  	bs_list = init_bs_list(MAX_BS_NUM);
  // print_schedule(bs_list);  

	printf(" Zycap Initialised \n");
 
     fd = fopen("/dev/zycap-vivado", "w+");
    if(fd < 0)
    {
       printf("\n cannot get the file descriptor \n");
       return -1;
    }
    else 
    {
      printf("\n Zycap Opened \n");
    }
    glbs.memfd = memfd;
    glbs.fd = fd;
     
	return memfd;	
}
	
void cmpr (char *buffer1, char *buffer2) 
{
    int ret;
    ret = strcmp (buffer1, buffer2);
    if (ret == 0) 
       printf("Both reads are identical\r\n");
    else 
       printf("Rubbissh!!\r\n");
}
   
    
int Config_PR_Bitstream(char *bs_name)
{
    int bs_pres;
    int pres_first;
    int pres_last;
    int size;
	int bytesMoved;
    bs_pres = check_bs_present(bs_list,bs_name);
	char fname[100];
	FILE *fd,*fp;
	char *buffersaved;
    if (bs_pres != -1 && bs_list[bs_pres].prev != NULL)            //The bitstream is already in the list and is not the most recently used
    {
    //	printf("bitstream in the list\n\r");
        bs_list[bs_pres].prev->next = bs_list[bs_pres].next;
        if(bs_list[bs_pres].next != NULL)
        {
            bs_list[bs_pres].next->prev = bs_list[bs_pres].prev;
        }
        pres_first = find_first_bs(bs_list);
        bs_list[bs_pres].prev = NULL;
        bs_list[bs_pres].next = &bs_list[pres_first];
        bs_list[pres_first].prev = &bs_list[bs_pres];
    }
    else if(bs_pres == -1)
    {
    //	printf("bitstream not in the list\n\r");
    	pres_last = find_last_bs(bs_list);
		strcpy (fname,bs_name);
		strcat (fname,".bin");
		fp = fopen(fname,"rb");
		if(!fp){
			printf("Unable to open PR file\n");
			return -1;
		}
    //printf("Reading PR file from flash\n\r");
		fseek(fp,0,SEEK_END);
		size=ftell(fp);
		fseek(fp,0,SEEK_SET);
		

		fread(bufferIn[pres_last].bitbuffer,1,size,fp);
    //fseek(fp,0,SEEK_SET);
    //buffersaved = (char *) malloc(size+1);    
	  //fread(buffersaved,1,size,fp);
    close(fp); 
		//printf("Successfully transferred PR files from flash to DDR\n\r");
    //cmpr(buffersaved, bufferIn[pres_last].bitbuffer);
		pres_first = find_first_bs(bs_list);
		bs_list[pres_last].prev->next = NULL;   //make the second last element as the last element
		strcpy(bs_list[pres_last].name,bs_name);
		bs_list[pres_last].size = size;
		bs_list[pres_last].prev = NULL;
		bs_list[pres_last].next = &bs_list[pres_first];
		bs_list[pres_first].prev = &bs_list[pres_last];
	}
//   print_schedule(bs_list);
    pres_first = find_first_bs(bs_list);
    //printf("Current bs %s , Size %d, Address %x \r\n",bs_list[pres_first].name,bs_list[pres_first].size,bs_list[pres_first].addr);
	fd = glbs.fd;
  unsigned long timestart = gettime();
	bytesMoved = memCpy_DMA(fd,bufferIn[pres_first].bitbuffer,bs_list[pres_first].size);
//	unsigned long timeend = gettime();
//  unsigned long time = timeend - timestart;
//  printf("%s\t%ld\t%ld\t%f\t\n", "DMA:Op", bs_list[pres_first].size, time, bs_list[pres_first].size * 1.0 / time);
	return bytesMoved;
}	

int conf_bit (char *bs_name)
{
    
    int size;
	int bytesMoved;
	char fname[100];
	FILE *fd,*fp;
	char *buffersaved;    
    
		strcpy (fname,bs_name);
		strcat (fname,".bin");
		fp = fopen(fname,"rb");
		if(!fp){
			printf("Unable to open PR file\n");
			return -1;
		}
    printf("Reading PR file from flash\n\r");
		fseek(fp,0,SEEK_END);
		size=ftell(fp);
		fseek(fp,0,SEEK_SET);
		//bufferIn = (char *) malloc(size+1);
	
		/*if(!bufferIn){
			printf("Unable to allocate buffer for data\n");
			fclose(fp);
			return -1;
		}*/

		fread(buffersaved,1,size,fp);
    close(fp); 

		printf("Successfully transferred PR files from flash to DDR\n\r");
   
   	fd = fopen("/dev/zycap-vivado", "w+");
    if(fd < 0)
    {
       printf("\n cannot get the file descriptor \n");
       return -1;
    }	
   
  	bytesMoved = memCpy_DMA(fd,buffersaved,size);
    return bytesMoved;
}


int close_zycap ()
{
  int i;
	free (bs_list);
  free (bufferIn);
	return 1;
}

int Prefetch_PR_Bitstream(char *bs_name)
{
    int bs_pres;
    int pres_first;
    int pres_last;
    int size;
	  FILE *fd,*fp;   
	  char fname[100];     
    bs_pres = check_bs_present(bs_list,bs_name);
    if (bs_pres != -1 && bs_list[bs_pres].prev != NULL)            //The bitstream is already in the list and is not the most recently used
    {
        bs_list[bs_pres].prev->next = bs_list[bs_pres].next;
        if(bs_list[bs_pres].next != NULL)
        {
            bs_list[bs_pres].next->prev = bs_list[bs_pres].prev;
        }
        pres_first = find_first_bs(bs_list);
        bs_list[bs_pres].prev = NULL;
        bs_list[bs_pres].next = &bs_list[pres_first];
        bs_list[pres_first].prev = &bs_list[bs_pres];
    }
    else if(bs_pres == -1)
    {
    	pres_last = find_last_bs(bs_list);
    	strcpy (fname,bs_name);
		  strcat (fname,".bin");
		  fp = fopen(fname,"rb");
		  if(!fp){
		  	printf("Unable to open PR file\n");
			  return -1;
		  }
    //printf("Reading PR file from flash\n\r");
		  fseek(fp,0,SEEK_END);
		  size=ftell(fp);
		  fseek(fp,0,SEEK_SET);

		  fread(bufferIn[pres_last].bitbuffer,1,size,fp);
      close(fp); 
	  	//printf("Successfully transferred PR files from flash to DDR\n\r");
      //cmpr(buffersaved, bufferIn[pres_last].bitbuffer);
	  	pres_first = find_first_bs(bs_list);
  		bs_list[pres_last].prev->next = NULL;   //make the second last element as the last element
	  	strcpy(bs_list[pres_last].name,bs_name);
  		bs_list[pres_last].size = size;
  		bs_list[pres_last].prev = NULL;
  		bs_list[pres_last].next = &bs_list[pres_first];
  		bs_list[pres_first].prev = &bs_list[pres_last];
    }
	return 1;
}
