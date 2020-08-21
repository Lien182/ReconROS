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

uint32_t calc_hash(uint32_t * data, uint32_t size)
{
	uint32_t hash = 0;
	
	for(int i = 0; i < size; i++)
		hash ^= data[i];
		
	return hash;
}

void *rt_sortdemo(void *data) {
	
	uint32_t * sortdata = (uint32_t*)malloc(sizeof(uint32_t) * BLOCK_SIZE);

	uint32_t hash;

	while (1) {
		
		printf("Generate new random data! \n");
		gen_random_data(sortdata, BLOCK_SIZE);
		hash = calc_hash(sortdata, BLOCK_SIZE);

		resources_sort_srv_req->unsorted.data = sortdata;
		resources_sort_srv_req->unsorted.size = BLOCK_SIZE;
		resources_sort_srv_req->unsorted.capacity = BLOCK_SIZE;
		
		printf("Send request \n");
		//Send random data to the service server and wait
		ROS_SERVICECLIENT_SEND(resources_srv, resources_sort_srv_req);
		
		printf("Wait for response \n");		
		ROS_SERVICECLIENT_TAKE(resources_srv, resources_sort_srv_res);

		//check whether data is sorted or not
		if(check_is_sorted(resources_sort_srv_res->sorted.data, BLOCK_SIZE) && calc_hash(resources_sort_srv_res->sorted.data, BLOCK_SIZE) == hash)
			printf("Data is correct! \n");
		else
			printf("Data is NOT correct! \n");
		
		
	}
}
