/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_service_server.h"
#include "rmw/types.h"

#include "../utils.h"

int ros_service_server_init(struct ros_service_server_t *ros_service_server, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* service_name, uint32_t wait_time)
{

    ros_service_server->node = &ros_node->node;
    ros_service_server->wait_time = wait_time;

    rcl_ret_t ret = 0;
    rcl_service_options_t service_ops = rcl_service_get_default_options();

    ret = rcl_service_init(
        &ros_service_server->service,
        ros_service_server->node,
        srv_type,
        service_name,
        &service_ops);

    if (RCL_RET_OK != ret) 
    {
        panic("[ROS Service Server]  Error in rcl_service_init %s; error code: %d \n", service_name, ret);
        return -1;
    }
    return ret;
}


int ros_service_server_destroy(struct ros_service_server_t *ros_service_server)
{
    return rcl_service_fini(&ros_service_server->service, ros_service_server->node);
}



int ros_service_server_try_take_request(struct ros_service_server_t *ros_service_server, void * req)
{
    rcl_ret_t rc;
    
    rc = rcl_take_request(
        &ros_service_server->service,
        &ros_service_server->request_id,
        req);

    
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SERVICE_TAKE_FAILED)
        {
            debug("[ROS Service Server] Error number: %d\n", rc);
            return -1;
        }
        else
        {
            //debug("[ROS Service Server] Return code : %d\n", rc);
            return 1;
        }        
    }

    return 0;
}


int ros_service_server_take_request(struct ros_service_server_t *ros_service_server, void * req)
{
    while(ros_service_server_try_take_request(ros_service_server, req) != 0)
        usleep(ros_service_server->wait_time);

    return 0;
}

int ros_service_server_send_response(struct ros_service_server_t *ros_service_server, void * res)
{
    rcl_ret_t rc;

    rc = rcl_send_response(
        &ros_service_server->service,
        &ros_service_server->request_id, 
        res);

    if(rc != RCL_RET_OK)
    {
        printf("[ROS Service Server] Error sending response: %d\n", rc);
        return -1;
    }
    
    return rc;
}

