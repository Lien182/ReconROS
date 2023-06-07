<<reconos_preproc>>
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "hls_stream.h"
#include <ap_int.h>

using namespace std;

void Send(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, uint64_t pMessage, hls::stream<t_stream> &hwtopic_out)
{
    sensor_msgs__msg__Image msg;
    ROS_PUBLISH_HWTOPIC_PACKED_FROM_MEMORY_<<HWTOPIC>>_out(hwtopic_out,  pMessage, msg);
}

void Clear(hls::stream<t_stream> &hwtopic_in)
{
    ROS_CLEAR_HWTOPIC_PACKED_SIZE_<<HWTOPIC>>_in( hwtopic_in);
}

int PS2PL(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, uint64_t pMessage, hls::stream<t_stream> &hwtopic_in, hls::stream<t_stream> &hwtopic_out)
{
    uint64_t ret = 0;    
    #pragma HLS dataflow
        
    Send(memif_hwt2mem, memif_mem2hwt, pMessage, hwtopic_out);
    Clear(hwtopic_in);
    return ret;
}


int PL2PS(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, uint64_t pMessageOut, hls::stream<t_stream> &hwtopic_in)
{
    uint64_t ret = 0;

    sensor_msgs__msg__Image msg;    
    ROS_TRYREAD_HWTOPIC_PACKED_TO_MEMORY_NEW_<<HWTOPIC>>_in(hwtopic_in, msg, pMessageOut, ret);
    
    return ret;
}


THREAD_ENTRY() {
    #pragma HLS INTERFACE axis port=<<HWTOPIC>>_in register
    #pragma HLS INTERFACE axis port=<<HWTOPIC>>_out register

    uint64_t fsm = 0;
    uint64_t ret = 0;
    uint64_t u64MsgPtr_PS2PL = 0;
    
    THREAD_INIT();
	uint64_t u64Initdata = GET_INIT_DATA();
    uint64_t u64MsgPtr_PL2PS = MEMORY_GETOBJECTADDR(<<RESOURCEGROUP>>_msg_out);

    while(1)
    {

        switch(fsm)
        {
            case 0: 
                ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_START(<<RESOURCEGROUP>>_sub, <<RESOURCEGROUP>>_msg_in);
                fsm = 1;
                break;

            case 1:
                if(ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_CHECK(u64MsgPtr_PS2PL))
                    fsm = 2;
                else
                    fsm = 3;
                break;

            case 2:
                PS2PL(memif_hwt2mem, memif_mem2hwt, u64MsgPtr_PS2PL, <<HWTOPIC>>_in, <<HWTOPIC>>_out);     
                fsm = 0;
                break;
            
            case 3:
                ret = PL2PS(memif_hwt2mem, memif_mem2hwt, u64MsgPtr_PL2PS, <<HWTOPIC>>_in);
                if(ret)
                {
                    ROS_SUBSCRIBE_TAKE_INTERRUPTABLE_CANCEL(u64MsgPtr_PS2PL,ret);
                    ROS_PUBLISH(<<RESOURCEGROUP>>_pub, <<RESOURCEGROUP>>_msg_out);
                    if(ret)
                        fsm = 2;
                    else
                        fsm = 0;
                    break;
                    
                }
                fsm = 1;
                break;
        }
     

    }

    return;
}
