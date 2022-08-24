#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

#define FRAME_ID_SIZE 5
#define ENCODING_SIZE 5
#define DATA_SIZE 640 * 480 * 3
#define MEM_STEP 4

t_stream tmpdata;

THREAD_ENTRY() {

//	#pragma HLS INTERFACE axis port=nicehwtopic
//	#pragma HLS INTERFACE axis port=verynicehwtopic

	//RAM(uint32_t, BLOCK_SIZE, ram);
	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint32_t payload_addr[1];

	sensor_msgs__msg__Image image_msg;

	THREAD_INIT();
	initdata = GET_INIT_DATA();


	uint32_t ram[64];
	uint32_t address_offset = 0;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		uint32_t msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);

		// copy msg from extern memory into local memory
		//MEM_READ(msg, ram, 64*4); // 4-bit boundary!
		MEM_READ(msg, &image_msg.header.stamp.sec, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.header.stamp.nanosec, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_READ(msg+address_offset, &image_msg.header.frame_id.size, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.header.frame_id.capacity, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, image_msg.header.frame_id.data, MEM_STEP*FRAME_ID_SIZE);
		address_offset += FRAME_ID_SIZE * MEM_STEP;

		MEM_READ(msg+address_offset, &image_msg.height, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.width, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_READ(msg+address_offset, &image_msg.encoding.size, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.encoding.capacity, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, image_msg.encoding.data, ENCODING_SIZE*MEM_STEP);
		address_offset += ENCODING_SIZE * MEM_STEP;

		MEM_READ(msg+address_offset, &image_msg.is_bigendian, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.step, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_READ(msg+address_offset, &image_msg.data.size, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, &image_msg.data.capacity, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_READ(msg+address_offset, image_msg.data.data, DATA_SIZE*MEM_STEP);
		address_offset += DATA_SIZE * MEM_STEP;

//		ROS_PUBLISH_HWTOPIC_nicehwtopic(&nicehwtopic, image_msg);

	}
}

