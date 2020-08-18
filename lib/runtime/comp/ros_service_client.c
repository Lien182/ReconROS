/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_service_client.h"
#include "rmw/types.h"

#include "../utils.h"


bool
wait_for_server_to_be_available(
  rcl_node_t * node,
  rcl_client_t * client,
  size_t max_tries,
  int64_t period_ms)
{
  size_t iteration = 0;
  while (iteration < max_tries) {
    ++iteration;
    bool is_ready;
    rcl_ret_t ret = rcl_service_server_is_available(node, client, &is_ready);
    if (ret != RCL_RET_OK) {
      return false;
    }
    if (is_ready) {
      return true;
    }
    usleep(period_ms*1000);
  }
  return false;
}

int ros_service_client_init(struct ros_service_client_t *ros_service_client, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* service_name, uint32_t wait_time)
{

    ros_service_client->node = &ros_node->node;
    ros_service_client->wait_time = wait_time;

    rcl_ret_t ret = 0;
    rcl_client_options_t service_ops = rcl_client_get_default_options();

    ros_service_client->service = rcl_get_zero_initialized_client();

    ret = rcl_client_init(
        &ros_service_client->service,
        ros_service_client->node,
        srv_type,
        service_name,
        &service_ops);

    if (RCL_RET_OK != ret) 
    {
        panic("[ROS Service Client]  Error in rcl_service_init %s; error code: %d \n", service_name, ret);
        return -1;
    }

    return ret;
}


int ros_service_client_destroy(struct ros_service_client_t *ros_service_client)
{
    return rcl_client_fini(&ros_service_client->service, ros_service_client->node);
}


int ros_service_client_request_send(struct ros_service_client_t *ros_service_client, void * req)
{
    rcl_ret_t rc;

    debug("[ROS Service Client] Send request; sequence number before: %lld \n", ros_service_client->request_id.sequence_number);



    if(wait_for_server_to_be_available( ros_service_client->node, &ros_service_client->service, 100, 1000) == false)
    {
         debug("[ROS Service Client] Server NOT available! \n");
         return -1;
    }


    rc = rcl_send_request(
        &ros_service_client->service,
        req,
        &ros_service_client->request_id.sequence_number);

    if(rc != RCL_RET_OK)
    {
        debug("[ROS Service Client] Error sending response: %d\n", rc);
        return -1;
    }
    else
    {
        debug("[ROS Service Client] Everthing ok, sending response: %d, sequence number %lld \n", rc, ros_service_client->request_id.sequence_number);
    }
    
    return rc;
}

int ros_service_client_response_try_take(struct ros_service_client_t *ros_service_client, void * res)
{
    rcl_ret_t rc;

    
    rc = rcl_take_response(
        &ros_service_client->service,
        &ros_service_client->request_id,
        res);

    
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SERVICE_TAKE_FAILED)
        {
            debug("[ROS Service Client] Error number: %d\n", rc);
            return -1;
        }
        else
        {
            //debug("[ROS Service client] Return code : %d\n", rc);
            return 1;
        }        
    }

    return 0;
}


int ros_service_client_response_take(struct ros_service_client_t *ros_service_client, void * res)
{
    while(ros_service_client_response_try_take(ros_service_client, res) != 0)
        usleep(ros_service_client->wait_time);

    return 0;
}