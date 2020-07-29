#include "reconos_calls.h"
#include "reconos_thread.h"
#include "ap_cint.h"

#include <sorter_msgs/srv/sort.h>

#define BLOCK_SIZE 2048



THREAD_ENTRY() {
	RAM(uint32, BLOCK_SIZE, ram);
	uint32 addr, initdata;	
	uint32 pMessage;
	uint32 payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	addr = ROS_MESSAGE_ARRAY_SET_SIZE(resources_sort_srv_req,  OFFSETOF(sorter_msgs__srv__Sort_Request, unsorted),  BLOCK_SIZE * 4);


	while(1) {

		for(int i = 0; i < BLOCK_SIZE; i++)
			ram[i] = BLOCK_SIZE - i;

		MEM_WRITE(ram, addr, BLOCK_SIZE * 4);

		ROS_SERVICECLIENT_SEND_REQUEST(resources_srv, resources_sort_srv_req );


		//Wait for response
		pMessage = ROS_SERVICESERVER_TAKE(resources_srv,resources_sort_srv_res);
		pMessage += OFFSETOF(sorter_msgs__srv__Sort_Request, unsorted.data);

		MEM_READ(pMessage, payload_addr, 4);					//Get the address of the data
		MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

		
		
		
		
	}
}
