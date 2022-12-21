#include "reconos_calls.h"
#include "reconos_thread.h"

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define MEM_STEP 8
#define DATA_SIZE 100 * 100 * 3


t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic

	uint64_t addr, initdata;	
	uint64_t pMessage;
	uint64_t payload_addr[1];
	uint64_t payload[1];

	sensor_msgs__msg__Image image_msg;

	uint8_t image_data[DATA_SIZE];
	#pragma HLS array_partition cyclic factor=4 variable=image_data
	char encoding[ENCODING_SIZE];
	char frame_id[FRAME_ID_SIZE];

	image_msg.data.data = image_data;
	image_msg.encoding.data = encoding;
	image_msg.header.frame_id.data = frame_id;

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	ap_axis<64,1,1,1> tmp_frame;

	uint64_t msg;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);
		
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + msg, payload_addr,     8);
		MEM_READ_INT8(payload_addr[0],image_msg.data.data, 30000)

		ROS_PUBLISH_HWTOPIC_v2_timing_nicehwtopic(nicehwtopic, image_msg);


	}
}

