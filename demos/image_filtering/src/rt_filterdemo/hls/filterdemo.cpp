#include "reconos_calls.h"
#include "reconos_thread.h"

#include "ap_cint.h"

#define BLOCK_SIZE 16

THREAD_ENTRY() {
	RAM(uint32, BLOCK_SIZE, ram);
	uint32 initdata;
	
	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		uint32 msg_ptr = ROS_SUBSCRIBE_TAKE(resources_subdata, resources_image_msg );
		MEM_READ(msg_ptr, ram, BLOCK_SIZE);
		MEM_WRITE(ram, msg_ptr, BLOCK_SIZE);
		ROS_PUBLISH(resources_pubdata,resources_image_msg);
	}
}
