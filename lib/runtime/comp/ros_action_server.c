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
    ros_action_server->wait_time = wait_time;

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
    rcl_ret_t ret = 0;

    ret |= rcl_action_server_fini(&ros_action_server->action, ros_action_server->node);
    ret |= rcl_clock_fini(&ros_action_server->clock);

    return ret;
}





int ros_action_server_goal_try_take(struct ros_action_server_t *ros_action_server, void * req)
{
    rcl_ret_t rc;
    
    rc = rcl_action_take_goal_request(
        &ros_action_server->action,
        &ros_action_server->request_id,
        req);

    
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SERVICE_TAKE_FAILED)
        {
            debug("[ROS Action Server] Error number: %d\n", rc);
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


int ros_action_server_goal_take(struct ros_action_server_t *ros_action_server, void * req)
{
    while(ros_action_server_goal_try_take(ros_action_server, req) != 0)
        usleep(ros_action_server->wait_time);

    return 0;
}

typedef struct 
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} _dummy_sendgoal_response;


int ros_action_server_goal_decide(struct ros_action_server_t *ros_action_server, uint32_t accept)
{
    rcl_ret_t ret = 0;

    _dummy_sendgoal_response sendgoal_resp;

    if(accept == 1)
    {
        rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
        // ... populate goal_info.uuid (unique_identifier_msgs/UUID)
        // ... populate goal_info.stamp (builtin_interfaces/Time)
        rcl_action_goal_handle_t * goal_handle = rcl_action_accept_new_goal(&ros_action_server->action, &goal_info);
        // ... error_handling
        // ... Populate goal response (client library type)

        // ... error handling, and sometime before shutdown finalize goal info message
        //ret = rcl_action_goal_info_fini(&goal_info, &ros_action_server->action);

        sendgoal_resp.accepted = 1;
    }
    else
    {
        /* code */
        sendgoal_resp.accepted = 0;
    }

    ret = rcl_action_send_goal_response(
            &ros_action_server->action,
            &ros_action_server->request_id,
            &sendgoal_resp);


    return ret;
    
}

int ros_action_server_feedback(struct ros_action_server_t *ros_action_server, void*  feedback_msg)
{
    rcl_ret_t rc = 0;

    rc = rcl_action_publish_feedback(
            &ros_action_server->action,
            feedback_msg);

    if (rc == RCL_RET_OK) 
    {
        printf("[ROS Action Server] Published feedback!\n");
        return 0;
    }
    else 
    {
        printf("[ROS Action Server] Error publishing feedback!\n");
        return rc;
    }
}

typedef struct
{
  unique_identifier_msgs__msg__UUID goal_id;
} _dummy_GetResult_Request; 
    

int ros_action_server_result_try_take(struct ros_action_server_t *ros_action_server)
{
    rcl_ret_t rc = 0;

    _dummy_GetResult_Request res_request;

    rc = rcl_action_take_result_request(
        &ros_action_server->action,
        &ros_action_server->request_id,
        &res_request);

    return rc;
    
}


int ros_action_server_result_take(struct ros_action_server_t *ros_action_server)
{
    while(ros_action_server_result_try_take(ros_action_server) != 0)
        usleep(ros_action_server->wait_time);

    return 0;
}


int ros_action_server_result_send(struct ros_action_server_t *ros_action_server, void * goal_res)
{
    rcl_ret_t rc = 0;

    rc = rcl_action_send_result_response(
        &ros_action_server->action,
        &ros_action_server->request_id,
        goal_res);

    return rc;
}


