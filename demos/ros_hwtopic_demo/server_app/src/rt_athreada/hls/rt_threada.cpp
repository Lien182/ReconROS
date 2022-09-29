#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>
#include <sensor_msgs/msg/image.h>

#include "hls_stream.h"

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define DATA_SIZE 320 * 240 * 3
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

	while(1) {

		ROS_READ_HWTOPIC_nicehwtopic(nicehwtopic, &image_msg);
		

		// copy message into external memory for publishing
		uint64_t output_buffer_addr = MEMORY_GETOBJECTADDR(rthreada_img_output);
		
		//MEM_READ(output_buffer_addr,payload_addr, 8 ); // img_msg.data.data is a pointer, so we need 2 reads in total
		//tmp_frame.data = image_msg.header.stamp.sec;
		//payload_addr[0] = tmp_frame.data;
		//MEM_WRITE(payload_addr, output_buffer_addr, 8);

		
		payload[0] = image_msg.header.stamp.sec;
		MEM_WRITE(payload, output_buffer_addr, MEM_STEP);
		address_offset += MEM_STEP;
		/*
		payload[0] = image_msg.header.stamp.nanosec;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		*/
		// frame_id-string
		
		payload[0] = image_msg.header.frame_id.size;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		/*
		payload[0] = image_msg.header.frame_id.capacity;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		
		MEM_READ(output_buffer_addr + address_offset, payload_address, MEM_STEP);		//Get the address of the data
		MEM_WRITE(image_msg.header.frame_id.data, payload_address[0], MEM_STEP * FRAME_ID_SIZE);							
		address_offset += MEM_STEP;		
		// possible problem: if the pointer to the array doesnt point to a location outside of the message 
		//
		
		payload[0] = image_msg.height;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		payload[0] = image_msg.width;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		// encoding-array
		payload[0] = image_msg.encoding.size;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		payload[0] = image_msg.encoding.capacity;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		
		MEM_READ(output_buffer_addr + address_offset, payload_address, MEM_STEP);		//Get the address of the data
		MEM_WRITE(image_msg.encoding.data, payload_address[0], MEM_STEP * ENCODING_SIZE);							
		address_offset += MEM_STEP;	
		//

		payload[0] = image_msg.is_bigendian;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		payload[0] = image_msg.step;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;


		// data-array
		payload[0] = image_msg.data.size;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		payload[0] = image_msg.data.capacity;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;

		MEM_READ(output_buffer_addr + address_offset, payload_address, MEM_STEP);		//Get the address of the data
		MEM_WRITE(image_msg.data.data, payload_address[0], MEM_STEP * DATA_SIZE);							
		address_offset += MEM_STEP;	
		//
		*/

		ROS_PUBLISH(rthreada_pubdata, rthreada_img_output);
	}
}
