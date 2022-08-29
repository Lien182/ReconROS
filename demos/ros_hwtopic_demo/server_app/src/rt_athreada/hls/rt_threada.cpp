#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>
#include <sensor_msgs/msg/image.h>

#include "hls_stream.h"

#define BLOCK_SIZE 2048

#define FRAME_ID_SIZE 5
#define ENCODING_SIZE 5
#define DATA_SIZE 640 * 480 * 3 
#define MEM_STEP 8

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
	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint64_t payload_addr[1];

	uint8_t image_data[200];

	//sensor_msgs__msg__Image image_msg;

	//image_msg.data.size = 200;
	//image_msg.data.capacity = 200;
	//image_msg.data.data = image_data;
	

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	ap_axis<32,1,1,1> tmp_frame;

	uint32_t address_offset = 0;

	while(1) {

		//ROS_READ_HWTOPIC_nicehwtopic(&nicehwtopic, image_msg);
	
		tmp_frame = nicehwtopic.read();


		// copy message into external memory for publishing
		uint64_t output_buffer_addr = MEMORY_GETOBJECTADDR(rthreada_img_output);
		MEM_READ(output_buffer_addr,payload_addr, 8 );
		payload_addr[0] = tmp_frame.data;
		MEM_WRITE(payload_addr, output_buffer_addr, 8);
/*
		//MEM_WRITE(&image_msg.header.stamp.sec, output_buffer_addr, MEM_STEP);
		MEM_WRITE(&image_msg.header.stamp.sec, output_buffer_addr, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.header.stamp.nanosec, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		
		MEM_WRITE(&image_msg.header.frame_id.size, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.header.frame_id.capacity, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(image_msg.header.frame_id.data, output_buffer_addr + address_offset, MEM_STEP*FRAME_ID_SIZE);
		address_offset += FRAME_ID_SIZE * MEM_STEP;

		MEM_WRITE(&image_msg.height, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.width, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_WRITE(&image_msg.encoding.size, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.encoding.capacity, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(image_msg.encoding.data, output_buffer_addr + address_offset, MEM_STEP*ENCODING_SIZE);
		address_offset += ENCODING_SIZE * MEM_STEP;

		MEM_WRITE(&image_msg.is_bigendian, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.step, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_WRITE(&image_msg.data.size, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(&image_msg.data.capacity, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		MEM_WRITE(image_msg.data.data, output_buffer_addr + address_offset, MEM_STEP*DATA_SIZE);
		address_offset += DATA_SIZE * MEM_STEP;
*/


		ROS_PUBLISH(rthreada_pubdata, rthreada_img_output);
	}
}
