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
	uint64_t payload_address[1];
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

	uint32_t ram[64];
	uint64_t address_offset = 0;

	uint64_t msg;

	uint64_t output_buffer_addr;
	output_buffer_addr = MEMORY_GETOBJECTADDR(rthreadb_img_output_hw);

	uint64_t mbox_value;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);
		
		MEM_READ(msg, payload, MEM_STEP);
		image_msg.header.stamp.sec = payload[0];
		address_offset += MEM_STEP;

		
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + msg, payload_address,     8);
		MEM_READ_INT8(payload_address[0],image_msg.data.data, DATA_SIZE)

		// hwtopic replacement
		mbox_value = MBOX_GET(rthreadb_start_mbox);

		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr +mbox_value, payload_address,     8);
		MEM_WRITE_INT8(image_msg.data.data,payload_address[0],DATA_SIZE)
		ROS_PUBLISH(rthreadb_pubdata2, rthreadb_img_output_hw);

		tmp_frame.data = mbox_value;
		nicehwtopic.write(tmp_frame);




		// for standard ReconROS topic: MBOX_GET, MEM_WRITE, ROS_PUBLISH
		//ROS_PUBLISH_HWTOPIC_v2_nicehwtopic(nicehwtopic, &image_msg);
		//
	}
}

