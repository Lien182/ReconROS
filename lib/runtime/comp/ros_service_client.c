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

int ros_service_client_init(struct ros_service_client_t *ros_service_client, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* service_name, uint32_t wait_time)
{

    ros_service_client->node = &ros_node->node;
    ros_service_client->wait_time = wait_time;

    rcl_ret_t ret = 0;
    rcl_client_options_t service_ops = rcl_client_get_default_options();

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


int ros_service_client_send_request(struct ros_service_client_t *ros_service_client, void * req)
{
    rcl_ret_t rc;

    rc = rcl_send_request(
        &ros_service_client->service,
        &ros_service_client->request_id, 
        req);

    if(rc != RCL_RET_OK)
    {
        printf("[ROS Service Client] Error sending response: %d\n", rc);
        return -1;
    }
    
    return rc;
}

int ros_service_client_try_take_response(struct ros_service_client_t *ros_service_client, void * res)
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
            debug("[ROS Service client] Error number: %d\n", rc);
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


int ros_service_client_take_response(struct ros_service_client_t *ros_service_client, void * res)
{
    while(ros_service_client_try_take_response(ros_service_client, res) != 0)
        usleep(ros_service_client->wait_time);

    return 0;
}