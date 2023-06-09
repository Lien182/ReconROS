#include "reconos_calls.h"
#include "reconos_thread.h"

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 200
#define ENCODING_SIZE 200
#define DATA_SIZE 200


t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic

	uint64_t payload_addr[1];
	uint64_t payload[1];

	sensor_msgs__msg__Image image_msg;

	uint64_t image_data[DATA_SIZE];
	uint64_t encoding[ENCODING_SIZE];
	uint64_t frame_id[FRAME_ID_SIZE];

	image_msg.data.size = 200;
	image_msg.data.capacity = 200;
	image_msg.data.data = (uint8_t*)image_data;
	image_msg.encoding.size = 200;
	image_msg.encoding.capacity = 201;
	image_msg.encoding.data = (char*)encoding;
	image_msg.header.frame_id.size = 200;
	image_msg.header.frame_id.capacity = 201;
	image_msg.header.frame_id.data = (char*)frame_id;

	image_msg.width = 124;
	image_msg.height = 2341;

	THREAD_INIT();
	uint64_t initdata = GET_INIT_DATA();

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic
		uint64_t pMessage = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);				
		ROS_PUBLISH_HWTOPIC_PACKED_FROM_MEMORY_nicehwtopic(nicehwtopic, pMessage, image_msg);


	}
}

