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

	sensor_msgs__msg__Image image_msg;

	THREAD_INIT();
	initdata = GET_INIT_DATA();


	uint32_t ram[64];
	uint32_t address_offset = 0;

	while(1) {

		// HWThread b: subscribe to data from software-domain, publish to hwtopic

		uint64_t msg = ROS_SUBSCRIBE_TAKE(rthreadb_subdata, rthreadb_img_input);
		MEM_READ(msg, payload_addr, 8);
		ap_axis<32,1,1,1> tmp_frame;
		tmp_frame.data = payload_addr[0]; 
		nicehwtopic.write(tmp_frame);

	}
}

