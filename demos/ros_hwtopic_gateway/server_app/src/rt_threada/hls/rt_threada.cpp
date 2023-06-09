#include "reconos_calls.h"
#include "reconos_thread.h"

#include <sensor_msgs/msg/image.h>

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 200
#define ENCODING_SIZE 200
#define DATA_SIZE 200


THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic

	uint64_t payload_address[1];
	uint64_t ret;
	

	uint64_t image_data[DATA_SIZE];
	uint64_t encoding[ENCODING_SIZE];
	uint64_t frame_id[FRAME_ID_SIZE];
	//char 

	sensor_msgs__msg__Image image_msg;

	image_msg.data.size = 200;
	image_msg.data.capacity = 200;
	image_msg.data.data = (uint8_t*)image_data;
	image_msg.encoding.size = 200;
	image_msg.encoding.capacity = 201;
	image_msg.encoding.data = (char*)encoding;
	image_msg.header.frame_id.size = 200;
	image_msg.header.frame_id.capacity = 201;
	image_msg.header.frame_id.data = (char*)frame_id;
	
	THREAD_INIT();
	uint64_t initdata = GET_INIT_DATA();
	
	uint64_t output_buffer_addr = MEMORY_GETOBJECTADDR(rthreada_img_output);
	//MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);

	while(1) {

		//ROS_READ_HWTOPIC_PACKED_SIZE_nicehwtopic(nicehwtopic, image_msg);			

		//ROS_

		//MEM_WRITE(image_data,payload_address[0], image_msg.data.size);									
		
		ROS_TRYREAD_HWTOPIC_PACKED_TO_MEMORY_nicehwtopic(nicehwtopic, image_msg, output_buffer_addr, ret);
		if(ret)		
			//MBOX_PUT(rthreada_debug_mbox, 1);
			ROS_PUBLISH(rthreada_pubdata, rthreada_img_output);
	}
}
