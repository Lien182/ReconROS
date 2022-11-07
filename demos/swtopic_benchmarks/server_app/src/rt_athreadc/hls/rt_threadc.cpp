#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define DATA_SIZE 100 * 100 * 3
#define MEM_STEP 8 // in bytes


t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic

	RAM(uint32_t, BLOCK_SIZE, ram);

	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint64_t payload[1];
	uint64_t payload_address[1];

	uint8_t image_data[DATA_SIZE];
	#pragma HLS array_partition cyclic factor=4 variable=image_data
	char encoding[ENCODING_SIZE];
	char frame_id[FRAME_ID_SIZE];
	//char 

	sensor_msgs__msg__Image image_msg;

	//image_msg.data.size = 200;
	//image_msg.data.capacity = 200;
	image_msg.data.data = image_data;
	image_msg.encoding.data = encoding;
	image_msg.header.frame_id.data = frame_id;

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	ap_axis<64,1,1,1> tmp_frame;

	uint64_t address_offset = 0;
	uint32_t temp = 0;
	uint64_t output_buffer_addr;
	output_buffer_addr = MEMORY_GETOBJECTADDR(rthreadc_img_output);
	

	uint64_t msg;

	while(1) {

		
		msg = ROS_SUBSCRIBE_TAKE(rthreadc_subdata2, rthreadc_img_input_hw);
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + msg, payload_address,     8);
		MEM_READ_INT8(payload_address[0],image_msg.data.data, DATA_SIZE);

		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);
		MEM_WRITE_INT8(image_msg.data.data,payload_address[0],DATA_SIZE)
		
		ROS_PUBLISH(rthreadc_pubdata, rthreadc_img_output);
		nicehwtopic.read(tmp_frame);
	}
}
