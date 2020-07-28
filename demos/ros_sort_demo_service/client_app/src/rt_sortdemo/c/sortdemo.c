#include "reconos_thread.h"
#include "reconos_calls.h"

#define BLOCK_SIZE 2048

void gen_random_data(uint32_t * data, uint32_t size)
{
	srand(time(0));
	for(int i = 0; i < size; i++)
		data[i] = rand();
}

uint32_t check_is_sorted(uint32_t * data, uint32_t size)
{
	
	for(int i = 1; i < size; i++)
		if(data[i] < data[i-1])
			return 0;
		
	return 1;
}

void *rt_sortdemo(void *data) {
	
	uint32_t * sortdata = (uint32_t*)malloc(sizeof(uint32_t) * BLOCK_SIZE);


	while (1) {
		
		printf("Generate new random data! \n");
		gen_random_data(sortdata, BLOCK_SIZE);

		//Send random data to the service server and wait
		ROS_SERVICECLIENT_SEND(resources_srv, resources_sort_srv_req);
		
		ROS_SERVICECLIENT_TAKE(resources_srv, resources_sort_srv_res);

		check_is_sorted(resources_sort_srv_res->sorted.data, BLOCK_SIZE);
		
	}
}
