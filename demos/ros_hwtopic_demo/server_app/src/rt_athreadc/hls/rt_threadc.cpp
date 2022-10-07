#include "reconos_calls.h"
#include "reconos_thread.h"

//#include <std_msgs/msg/u_int32_multi_array__struct.h>

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define DATA_SIZE 320 * 240 * 3
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

	uint64_t address_offset = 0;
	uint32_t temp = 0;
	uint32_t tens = 10;

	while(1) {

		ROS_READ_HWTOPIC_nicehwtopic(nicehwtopic, &image_msg);
		

		// copy message into external memory for publishing
		uint64_t output_buffer_addr = MEMORY_GETOBJECTADDR(rthreadc_img_output);
		
		//MEM_READ(output_buffer_addr,payload_addr, 8 ); // img_msg.data.data is a pointer, so we need 2 reads in total
		//tmp_frame.data = image_msg.header.stamp.sec;
		//payload_addr[0] = tmp_frame.data;
		//MEM_WRITE(payload_addr, output_buffer_addr, 8);
		temp = 0;
		for(uint32_t i = 0; i < 8; ++i)
		{
			temp = temp * 10 + image_msg.data.data[i];
		}

		payload[0] = temp;
		//payload[0] = image_msg.header.stamp.sec;
		MEM_WRITE(payload, output_buffer_addr, MEM_STEP);
		address_offset += MEM_STEP;
		
		/*
		payload[0] = image_msg.header.stamp.nanosec;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		*/
		// frame_id-string
		/*
		payload[0] = image_msg.header.frame_id.size;
		MEM_WRITE(payload, output_buffer_addr + address_offset, MEM_STEP);
		address_offset += MEM_STEP;
		
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
		*/

		//MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);
		//MEM_WRITE(image_msg.data.data, payload_address[0], DATA_SIZE);


		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);
		MEM_WRITE_INT8(image_msg.data.data,payload_address[0],8)

		address_offset += MEM_STEP;	
		//
		

		ROS_PUBLISH(rthreadc_pubdata, rthreadc_img_output);
	}
}
