#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "ros.h"
#include "ros_sub.h"
#include "ros_pub.h"
#include <pthread.h>

#define MODE_PUBLISHER	1
#define MODE_SUBSCRIBER	2
#define MODE_NONE		99

pthread_mutex_t mutex;

struct ros_node_t 		resources_rosnode[2];
struct ros_subscriber_t resources_subdata[2];
struct ros_publisher_t 	resources_pubdata[2];

typedef struct {
	uint32_t	cnt;
	uint32_t 	mode;
	uint32_t 	wait_time;
	char* 		topic;
	char*		msg;
}t_thread_settings;


void my_handler (int sig)
{
	printf("Interrupt\n");
	ros_subscriber_destroy(resources_subdata);
  	ros_node_destroy(resources_rosnode);
	exit(0);
}




void* node_thread(void * arg)
{
	int i = 0;
	char * pub_msg;

	t_thread_settings* sett = (t_thread_settings*)arg;

	pthread_mutex_lock (&mutex);

	if(ros_node_init(&resources_rosnode[sett->cnt]) < 0)
	{
		printf("Thread %d: ROS Node init failed \n", sett->cnt);
		return -1;
	}
	pthread_mutex_unlock (&mutex);
		
	if(sett->mode == MODE_PUBLISHER)
	{
		if( ros_publisher_init(&resources_pubdata[sett->cnt], &resources_rosnode[sett->cnt], sett->topic, 100) < 0 )
		{
			ros_node_destroy(&resources_rosnode[sett->cnt]);
			return -1;
		}

		for(i = 0; i <20; i++ )
		{
			asprintf(&pub_msg, "[ReconROS_Node_%d, msg %d]: %s", resources_rosnode[sett->cnt].node_nr, i, sett->msg);
			printf("%s\n", pub_msg);		
			//ros_publisher_publish(&resources_pubdata[sett->cnt], (uint8_t*)sett->msg, strlen(sett->msg));
			
			ros_publisher_publish(&resources_pubdata[sett->cnt], (uint8_t*)pub_msg, strlen(pub_msg));
			
			free(pub_msg);

			usleep(sett->wait_time);
		}

		ros_publisher_destroy(&resources_pubdata[sett->cnt]);

	}
	else if(sett->mode == MODE_SUBSCRIBER)
	{
		ros_subscriber_init(&resources_subdata[sett->cnt], &resources_rosnode[sett->cnt], sett->topic, 100);

		for(i = 0; i <20; i++ )
		{
			//ros_publisher_publish(&resources_pubdata[sett->cnt], (uint8_t*)sett->msg, strlen(sett->msg));
			usleep(sett->wait_time);
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

	return 0;
}


int main(int argc, char **argv) 
{


	pthread_t p1, p2;

	t_thread_settings settings[2];

	pthread_mutex_init (&mutex, NULL);
	
	signal (SIGQUIT, my_handler);
	signal (SIGINT, my_handler);

	settings[0].cnt = 0;
	settings[0].mode = MODE_PUBLISHER;
	settings[0].wait_time = 100000;
	settings[0].msg = "test";
	settings[0].topic = "chatter";

	settings[1].cnt = 1;
	settings[1].mode = MODE_PUBLISHER;
	settings[1].wait_time = 100000;
	settings[1].msg = "test";
	settings[1].topic = "chatter";

	pthread_create(&p1, NULL, &node_thread, &settings[0]);
	pthread_create(&p2, NULL, &node_thread, &settings[1]);

  	pthread_join (p1, NULL);
  	pthread_join (p2, NULL);

	return 0;
}