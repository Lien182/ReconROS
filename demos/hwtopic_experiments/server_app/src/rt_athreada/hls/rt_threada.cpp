#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>
#include <sensor_msgs/msg/image.h>

#include "hls_stream.h"

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define DATA_SIZE 100 * 100 * 3
#define MEM_STEP 8 // in bytes

//extern rosidl_typesupport_introspection_c__MessageMembers_ Image__rosidl_typesupport_introspection_c__Image_message_members_;

t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE axis port=nicehwtopic
	#pragma HLS INTERFACE ap_fifo port=osif_sw2hw
	#pragma HLS INTERFACE ap_fifo port=osif_hw2sw
	#pragma HLS INTERFACE ap_fifo port=memif_hwt2mem
	#pragma HLS INTERFACE ap_fifo port=memif_mem2hwt
	
	//#pragma HLS INTERFACE axis port=verynicehwtopic

	//RAM(uint32_t, BLOCK_SIZE, ram);
	uint32_t addr, initdata;				// not 64 bit for ReconOS64?
	uint32_t pMessage;
	uint64_t payload[1];
	uint64_t payload_address[1];

	uint8_t image_data[DATA_SIZE];
	#pragma HLS array_partition cyclic factor=8 variable=image_data
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
	output_buffer_addr = MEMORY_GETOBJECTADDR(rthreada_img_output);
	MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);

	uint64_t buffer[30500];

	while(1) {

		// for standard ReconROS topic: ROS_SUBSCRIBE_TAKE, MEM_READ, MBOX_PUT
		//ROS_READ_HWTOPIC_v4_timing_nicehwtopic(nicehwtopic, image_msg);
		ROS_READ_HWTOPIC_v7_timing_nicehwtopic(nicehwtopic, image_msg);
		/*
		for(uint32_t i = 0; i < 30000; i++){
			nicehwtopic.read(tmp_frame);
			buffer[i] = tmp_frame.data;
		}
		for(uint32_t i = 0; i < 30000; i++){
			image_msg.data.data[i] = buffer[i];
		}
		*/

		
		MEM_WRITE_INT8(image_msg.data.data,payload_address[0],30000)
									
		ROS_PUBLISH(rthreada_pubdata, rthreada_img_output);
	}
}
