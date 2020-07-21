 /*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_action_server.h"
#include "rmw/types.h"

#include "../utils.h"

int ros_action_server_init(struct ros_action_server_t *ros_action_server, struct ros_node_t * ros_node, const rosidl_action_type_support_t * action_type, char* action_name, uint32_t wait_time)
{

    ros_action_server->node = &ros_node->node;

    rcl_ret_t ret = 0;

    rcl_action_server_options_t action_ops = rcl_action_server_get_default_options();


     
    ret = rcl_clock_init(RCL_STEADY_TIME, &ros_action_server->clock, &ros_node->allocator);
    if (RCL_RET_OK != ret) 
    {
        panic("[ROS Action Server]  Error in rcl_clock_init %s; error code: %d \n", action_name, ret);
        return -1;
    }
    
    ret = rcl_action_server_init(
        &ros_action_server->action,
        ros_action_server->node,
        &ros_action_server->clock,
        action_type,
        action_name,
        &action_ops);

    if (RCL_RET_OK != ret) 
    {
        panic("[ROS Action Server]  Error in rcl_action_server_init %s; error code: %d \n", action_name, ret);
        return -1;
    }
    return ret;
}


int ros_action_server_destroy(struct ros_action_server_t *ros_action_server)
{
    rcl_action_server_fini(&ros_action_server->action, ros_action_server->node);

    return rcl_clock_fini(&ros_action_server->clock);
}

/*


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

*/