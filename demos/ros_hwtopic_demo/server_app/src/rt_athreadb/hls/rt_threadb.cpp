#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define MEM_STEP 8
#define DATA_SIZE 320 * 240 * 3


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

	uint32_t ram[64];
	uint64_t address_offset = 0;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		uint64_t msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);

		uint64_t input_buffer_addr = MEMORY_GETOBJECTADDR(rthreadb_img_input);
		
		MEM_READ(msg, payload, MEM_STEP);
		image_msg.header.stamp.sec = payload[0];
		address_offset += MEM_STEP;
		/*
		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.header.stamp.nanosec = payload[0];
		address_offset += MEM_STEP;
		
		// frame_id string
		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.header.frame_id.size = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.header.frame_id.capacity = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload_addr, MEM_STEP);
		MEM_READ(payload_addr[0], image_msg.header.frame_id.data, FRAME_ID_SIZE);
		address_offset += MEM_STEP;
		//


		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.height = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.width = payload[0];
		address_offset += MEM_STEP;


		// encoding string
		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.encoding.size = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.encoding.capacity = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload_addr, MEM_STEP);
		MEM_READ(payload_addr[0], image_msg.encoding.data, ENCODING_SIZE);
		address_offset += MEM_STEP;
		//

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.is_bigendian = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.step = payload[0];
		address_offset += MEM_STEP;


		// data array
		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.data.size = payload[0];
		address_offset += MEM_STEP;

		MEM_READ(msg + address_offset, payload, MEM_STEP);
		image_msg.data.capacity = payload[0];
		address_offset += MEM_STEP;
		*/
		//MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + input_buffer_addr, payload_addr,     8);
		//MEM_READ(msg + address_offset, payload_addr, MEM_STEP);
		//MEM_READ(payload_addr[0], image_msg.data.data, DATA_SIZE);
		address_offset += MEM_STEP;
		
		image_msg.data.data[0] = 0;
		image_msg.data.data[1] = 1;
		image_msg.data.data[2] = 2;
		image_msg.data.data[3] = 3;
		image_msg.data.data[4] = 4;
		image_msg.data.data[5] = 5;
		image_msg.data.data[6] = 6;
		image_msg.data.data[7] = 7;
		//

		ROS_PUBLISH_HWTOPIC_nicehwtopic(nicehwtopic, &image_msg);


		/*
		ap_axis<64,1,1,1> tmp_frame;
		tmp_frame.data = payload_addr[0]; 
		nicehwtopic.write(tmp_frame);
		*/
	}
}

