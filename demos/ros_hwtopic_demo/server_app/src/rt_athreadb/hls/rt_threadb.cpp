#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define MEM_STEP 8
#define DATA_SIZE 100 * 100 * 3


t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic
//	#pragma HLS INTERFACE axis port=verynicehwtopic

	//RAM(uint32_t, BLOCK_SIZE, ram);
	uint64_t addr, initdata;	
	uint64_t pMessage;
	uint64_t payload_addr[1];
	uint64_t payload[1];

	sensor_msgs__msg__Image image_msg;

	uint8_t image_data[DATA_SIZE];
	char encoding[ENCODING_SIZE];
	char frame_id[FRAME_ID_SIZE];

	image_msg.data.data = image_data;
	image_msg.encoding.data = encoding;
	image_msg.header.frame_id.data = frame_id;

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	ap_axis<64,1,1,1> tmp_frame;

	uint32_t ram[64];
	uint64_t address_offset = 0;

	uint64_t msg;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);
		
		MEM_READ(msg, payload, MEM_STEP);
		image_msg.header.stamp.sec = payload[0];
		address_offset += MEM_STEP;

		
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + msg, payload_addr,     8);
		MEM_READ_INT8(payload_addr[0],image_msg.data.data, 30000)

		/*
		for(int32_t i = 5; i < 30000; ++i){
			image_msg.data.data[i] = i;
		}
		*/
		/*
		for (uint32_t i = 0; i < 30000; i++)
		{
			tmp_frame.data = image_msg.data.data[i];
			nicehwtopic.write(tmp_frame);
		}
		*/
		ROS_PUBLISH_HWTOPIC_nicehwtopic(nicehwtopic, &image_msg);


	}
}

