#include "reconos_calls.h"
#include "reconos_thread.h"

#include "ap_cint.h"


THREAD_ENTRY() {
	uint32 initdata;
	
	THREAD_INIT();
	initdata = GET_INIT_DATA();

	while(1) {

		ROS_SUBSCRIBE_TAKE(resources_subdata, resources_image_msg );
		ROS_PUBLISH(resources_pubdata,resources_image_msg);
	}
}
