#include "reconos_thread.h"
#include "reconos_calls.h"


void *rt_filterdemo(void *data) {
	
	printf("Now start the thread! \n");
	while (1) 
	{
		ROS_SUBSCRIBE_TAKE(resources_subdata, resources_image_msg);
		ROS_PUBLISH(resources_pubdata, resources_image_msg);
	}
}
