#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#define debug(...)

#define SHOW_DATA 1

#include "../lib/comp/ros.h"
#include "../lib/comp/ros_sub.h"
#include "../lib/comp/ros_pub.h"


#define BLOCK_SIZE 	2048
#define ITERATIONS	10

#define MODE_PUBLISHER	1
#define MODE_SUBSCRIBER	2
#define MODE_NONE		99

#define ROS_TYPE_MESSAGE_TYPE rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32MultiArray()

#define OFFSETOF(type, member) ((uint32_t)(intptr_t)&(((type *)(void*)0)->member) )

pthread_mutex_t mutex;

struct ros_node_t 		resources_rosnode[2];
struct ros_subscriber_t resources_subdata[2];
struct ros_publisher_t 	resources_pubdata[2];

typedef struct {
	uint32_t	cnt;
	uint32_t 	mode;
	uint32_t 	wait_time;
	char* 		topic;
	char* 		nodename;
	uint8_t*	msg;
	uint32_t 	msg_length;
}t_thread_settings;


void my_handler (int sig)
{
	printf("Interrupt\n");
	ros_subscriber_destroy(resources_subdata);
  	ros_node_destroy(resources_rosnode);
	exit(0);
}

int check_data(uint32_t * data, uint32_t length)
{

#ifdef SHOW_DATA	
	for(int i = 0; i < 10; i+=1)
	{
		printf("%x ", data[i]);
		if(i % 50 == (50-1)) printf("\n");
	}
#endif	
	for(int i = 0; i < length-1; i++)
	{
		if(data[i+1] < data[i])
		{
			printf("unsorted at i=%d \n",i);
			return -1;
		}
	}
	printf("\n");
	return 0;
}

void clear_data(uint32_t * data, uint32_t length)
{
	for(int i = 0; i < length; i++)
	{
		data[i] = 0;
	}
}

void random_data(uint32_t * data, uint32_t length)
{
	for(int i = 0; i < length; i++)
	{
		data[i] = rand();
	}	
}

uint32_t xor_hash(uint32_t * data, uint32_t length)
{
	uint32_t ret = 0;
	for(int i = 0; i < length; i++)
	{
		ret ^= data[i];
	}

	return ret;
}

void std_msgs__msg__UInt32MultiArray_fill_one_dim(std_msgs__msg__UInt32MultiArray * array, uint32_t * data, uint32_t size)
{
	array->data.data = data;
	array->data.size = size;
	array->data.capacity = size;

	

	array->layout.data_offset = 0;
	array->layout.dim.size = 1;
	array->layout.dim.capacity = 1;

	
	array->layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(sizeof(std_msgs__msg__MultiArrayDimension));

	array->layout.dim.data->size = 1;
	array->layout.dim.data->stride = 1;

	array->layout.dim.data->label.data = (char*)malloc(sizeof(strlen("label"))+1);
	strcpy(array->layout.dim.data->label.data, "label");

	array->layout.dim.data->label.size = strlen("label");
	array->layout.dim.data->label.capacity = strlen("label")+1;
}

void* node_thread(void * arg)
{
	int i = 0;
	t_thread_settings* sett = (t_thread_settings*)arg;

	std_msgs__msg__UInt32MultiArray * sort_msg = std_msgs__msg__UInt32MultiArray__create();


	pthread_mutex_lock (&mutex);

	if(ros_node_init(&resources_rosnode[sett->cnt],sett->nodename) < 0)
	{
		printf("Thread %d: ROS Node init failed \n", sett->cnt);
		return (void*)-1;
	}
	pthread_mutex_unlock (&mutex);

	if(sett->mode == MODE_PUBLISHER)
	{
		if( ros_publisher_init(&resources_pubdata[sett->cnt], &resources_rosnode[sett->cnt], ROS_TYPE_MESSAGE_TYPE, sett->topic) < 0 )
		{
			ros_node_destroy(&resources_rosnode[sett->cnt]);
			return (void*)-1;
		}
		usleep(200000);	

		for(i = 0; i < ITERATIONS; i++ )
		{
			random_data((uint32_t*)sett->msg, BLOCK_SIZE);

			std_msgs__msg__UInt32MultiArray_fill_one_dim(sort_msg, (uint32_t*)sett->msg, BLOCK_SIZE );
			printf("[ReconROS_Node_%d_%d] Hash %x \n", sett->cnt,i, xor_hash((uint32_t*)sort_msg->data.data, BLOCK_SIZE));
			printf("[ReconROS_Node_%d_%d]", sett->cnt,i);
			check_data((uint32_t*)sort_msg->data.data, BLOCK_SIZE);
			ros_publisher_publish(&resources_pubdata[sett->cnt], sort_msg);
			usleep(sett->wait_time);
		}

		ros_publisher_destroy(&resources_pubdata[sett->cnt]);

	}
	else if(sett->mode == MODE_SUBSCRIBER)
	{
		ros_subscriber_init(&resources_subdata[sett->cnt], &resources_rosnode[sett->cnt],ROS_TYPE_MESSAGE_TYPE, sett->topic, 100000);

		

		for(i = 0; i < ITERATIONS; i++ )
		{
			
			ros_subscriber_message_take(&resources_subdata[sett->cnt], sort_msg);
			usleep(sett->wait_time);

			printf("[ReconROS_Node_%d_%d] Hash %x \n", sett->cnt,i, xor_hash((uint32_t*)sort_msg->data.data, BLOCK_SIZE));

			printf("[ReconROS_Node_%d_%d]", sett->cnt,i);
			if(check_data((uint32_t*)sort_msg->data.data, BLOCK_SIZE) != 0)
				printf("[ReconROS_Node_%d_%d] Data is unsorted! \n", sett->cnt,i);
			else
				printf("[ReconROS_Node_%d_%d] Data is sorted \n", sett->cnt,i);

			clear_data((uint32_t*)sort_msg->data.data, BLOCK_SIZE);
		}

		ros_subscriber_destroy(&resources_subdata[sett->cnt]);



	}
	else if(sett->mode == MODE_NONE)
	{
		for(i = 0; i < 10; i++)
		{
			printf("Sleep: %d \n", i);
			sleep(1);
		}
	}


	ros_node_destroy(&resources_rosnode[sett->cnt]);

	return (void*)0;
}


int main(int argc, char **argv) 
{

	uint32_t u32usorted[BLOCK_SIZE];
	uint32_t u32sorted[BLOCK_SIZE];

	pthread_t p1, p2;

	t_thread_settings settings[2];

	signal (SIGQUIT, my_handler);
	signal (SIGINT, my_handler);
	pthread_mutex_init (&mutex, NULL);
	
	srand(time(0));

	for(int i = 0; i < BLOCK_SIZE; i++)
	{
		u32usorted[i] = rand();
		u32sorted[i] = 0;
	}


	printf("Calculated Offset %d \n", OFFSETOF(std_msgs__msg__UInt32MultiArray, data.data));

	settings[0].cnt = 0;
	settings[0].mode = MODE_PUBLISHER;
	settings[0].wait_time = 1000000;
	settings[0].msg = (uint8_t*)u32usorted;
	settings[0].msg_length = BLOCK_SIZE * sizeof(uint32_t);
	settings[0].topic = "unsorted";
	settings[0].nodename = "node_1";
	

	settings[1].cnt = 1;
	settings[1].mode = MODE_SUBSCRIBER;
	settings[1].wait_time = 100000;
	settings[1].msg = (uint8_t*)u32sorted;
	settings[1].msg_length = BLOCK_SIZE * sizeof(uint32_t);
	settings[1].topic = "sorted";
	settings[1].nodename = "node_2";

	

	pthread_create(&p1, NULL, &node_thread, &settings[0]);
	pthread_create(&p2, NULL, &node_thread, &settings[1]);

  	pthread_join (p1, NULL);
  	pthread_join (p2, NULL);

	return 0;
}