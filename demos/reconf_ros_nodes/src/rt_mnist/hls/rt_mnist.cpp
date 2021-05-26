/********************************************************************          
* rt_sobel.cpp         -- hardware sobel implementation             *
*                         calculated the sobel operation for        *
*                         all three color channels                  * 
*																	* 
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#include "reconos_thread.h"
#include "reconos_calls.h"

#include <mnist_msgs/srv/mnist.h>

#include "ap_int.h"
#include "ap_fixed.h"
#include <math.h>

#include "LeNet.h"

#include "parameters.h"


#define INPUT_BATCH_SZ	1
#define INPUT_N_ROWS 28
#define INPUT_N_COLS 28


void read_image(uint32 ram[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS/4], uint8_t image[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS]){
	// Read the input stream and put the values in the image array

#pragma HLS INLINE off
#pragma HLS array_partition variable=image cyclic factor=4 dim=1

	int b, r, c, i = 0;
	uint32_t tmp;

	batch_rd: for(b=0; b<INPUT_BATCH_SZ; b++){
		row_rd: for (r=0; r<INPUT_N_ROWS; r++) {
			col_rd: for (c=0; c<INPUT_N_COLS; c+=4){

#pragma HLS PIPELINE
				tmp = ram[i];
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c]   = (uint8_t)((tmp & 0x0000ff));
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+1] = (uint8_t)((tmp & 0x0000ff00) >> 8);
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+2] = (uint8_t)((tmp & 0x00ff0000) >> 16); 		
				image[b*INPUT_N_COLS*INPUT_N_COLS + INPUT_N_COLS*r + c+3] = (uint8_t)((tmp & 0xff000000) >> 24); 
				i++;
			}
		}
	}

	return;
}


THREAD_ENTRY()
{

//#pragma HLS DATAFLOW

	uint8_t image[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS];
	ap_axis<HW_DATA_WIDTH,1,1,1> src[BUFFER_SIZE], dst[CLASSES];


	THREAD_INIT();

	uint32 output_buffer_addr = GET_INIT_DATA();


	while (1)
	{
		uint32 status = 0;
		uint32 payload_addr[1];
		
		uint32 ram[INPUT_BATCH_SZ*INPUT_N_ROWS*INPUT_N_COLS/4];

		uint32 pMessage= ROS_SERVICESERVER_TAKE(rmnist_srv, rmnist_mnist_srv_req );
		MEM_READ(OFFSETOF(mnist_msgs__srv__Mnist_Request, rawdigit.data.data) + pMessage, payload_addr, 4);
		MEM_READ( payload_addr[0], ram, INPUT_N_ROWS*INPUT_N_COLS);

		//memcpy(image, ram,  N_ROWS*N_COLS);

		read_image(ram, image);

		for(int i=0; i<1; i++)
		{
			char tmp;
			for(int batch=0; batch<1; batch++)
			{
				for(int rows = 0; rows < 32 ; rows++)
				{
					for(int cols = 0; cols < 32; cols++)
					{
						#pragma HLS pipeline
						ap_fixed<32,16> scaled;
						uint8_t temp;

						if(cols<2 || rows<2 || cols>=30|| rows>=30)
						{
							temp = 0;
						}
						else
						{
							temp = image[batch*1024+(rows-2)*28+cols-2];
						}

						scaled =  ((((ap_fixed<32,16>)temp * (ap_fixed<32,16>)2) / (ap_fixed<32,16>)255 )- (ap_fixed<32,16>)1 );
						ap_int<HW_DATA_WIDTH> tmp;
						tmp = (ap_int<HW_DATA_WIDTH>)(scaled*DATA_CONVERT_MUL);
						src[rows*INPUT_WH+cols].data = tmp;

						src[i].keep = 1;
						src[i].strb = 1;
						src[i].user = 1;
						src[i].last = 0;
						src[i].id = 0;
						src[i].dest = 1;
					}

				}

			}
		}
		
		LeNet(src, dst, 0);

		ap_fixed<32,16> result[CLASSES];

		ap_fixed<32,16> max_num = -10000;
		int max_id = 0;
		for(int index=0; index<10; index++){
			#pragma HLS pipeline
			int tmp = dst[index].data;
			result[index] = (ap_fixed<32,16>)tmp/(ap_fixed<32,16>)DATA_CONVERT_MUL;
			if(result[index] > max_num){
				max_num = result[index];
				max_id = index;
			}
		}




		uint32 output_digit[1];
		output_digit[0] = max_id;

		MEM_WRITE( output_digit, output_buffer_addr, 4);
		ROS_SERVICESERVER_SEND_RESPONSE(rmnist_srv, rmnist_mnist_srv_res);		
	}
}
