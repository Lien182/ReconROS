#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

#define FRAME_ID_SIZE 5
#define ENCODING_SIZE 5
#define DATA_SIZE 640 * 480 * 3
#define MEM_STEP 8

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

	uint8_t image_data[200];
	char encoding[5];
	char frame_id[10];

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

		MEM_READ(msg, payload, MEM_STEP);
		image_msg.header.stamp.sec = payload[0];
		address_offset += MEM_STEP;

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
		MEM_READ(payload_addr[0], image_msg.header.frame_id.data, MEM_STEP * FRAME_ID_SIZE);
		address_offset += MEM_STEP;
		//


		MEM_READ(msg, payload, MEM_STEP);
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
		MEM_READ(payload_addr[0], image_msg.encoding.data, MEM_STEP * ENCODING_SIZE);
		address_offset += MEM_STEP;
		//

		MEM_READ(msg, payload, MEM_STEP);
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

		MEM_READ(msg + address_offset, payload_addr, MEM_STEP);
		MEM_READ(payload_addr[0], image_msg.data.data, MEM_STEP * DATA_SIZE);
		address_offset += MEM_STEP;
		//

		ROS_PUBLISH_HWTOPIC_nicehwtopic(nicehwtopic, &image_msg);


		/*
		ap_axis<32,1,1,1> tmp_frame;
		tmp_frame.data = payload_addr[0]; 
		nicehwtopic.write(tmp_frame);
		*/
	}
}

