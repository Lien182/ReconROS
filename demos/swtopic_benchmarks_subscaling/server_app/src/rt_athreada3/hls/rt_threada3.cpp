#include "reconos_calls.h"
#include "reconos_thread.h"
#include <sensor_msgs/msg/image.h>
#include "hls_stream.h"

#define BLOCK_SIZE 2048

// size-definitions in byte, must be 8-byte aligned
#define FRAME_ID_SIZE 8
#define ENCODING_SIZE 8
#define DATA_SIZE 100 * 100 * 3
#define MEM_STEP 8 // in bytes

t_stream tmpdata;

THREAD_ENTRY() {

	#pragma HLS INTERFACE ap_fifo port=osif_sw2hw
	#pragma HLS INTERFACE ap_fifo port=osif_hw2sw
	#pragma HLS INTERFACE ap_fifo port=memif_hwt2mem
	#pragma HLS INTERFACE ap_fifo port=memif_mem2hwt

	uint32_t addr, initdata;		
	uint32_t pMessage;
	uint64_t payload[1];
	uint64_t payload_address[1];
	uint8_t image_data[DATA_SIZE];
	char encoding[ENCODING_SIZE];
	char frame_id[FRAME_ID_SIZE];
	sensor_msgs__msg__Image image_msg;
	image_msg.data.data = image_data;
	image_msg.encoding.data = encoding;
	image_msg.header.frame_id.data = frame_id;
	

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	uint64_t output_buffer_addr;
	output_buffer_addr = MEMORY_GETOBJECTADDR(rthreada3_img_output);
	uint64_t msg;
	uint64_t mbox_value = 1;

	while(1) {
		io_section : {
		#pragma HLS PROTOCOL fixed
		msg = ROS_SUBSCRIBE_TAKE(rthreada3_subdata2, rthreada3_img_input_hw);
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + msg, payload_address,     8);
		MEM_READ_INT8(payload_address[0],image_msg.data.data, DATA_SIZE);

		mbox_value = image_msg.data.data[0];
		MBOX_PUT(rthreada3_finish_mbox, mbox_value);
		
		ap_wait();
		MEM_READ(OFFSETOF(sensor_msgs__msg__Image, data.data) + output_buffer_addr, payload_address,     8);
		MEM_WRITE_INT8(image_msg.data.data,payload_address[0],DATA_SIZE)
		ap_wait();							
		ROS_PUBLISH(rthreada3_pubdata, rthreada3_img_output);
		
		}
	}
}
