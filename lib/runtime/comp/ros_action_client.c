 /*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_action_client.h"
#include "rmw/types.h"

#include "../utils.h"

static bool
wait_for_server_to_be_available(
  rcl_node_t * node,
  rcl_action_client_t * client,
  size_t max_tries,
  int64_t period_ms)
{
  size_t iteration = 0;
  while (iteration < max_tries) {
    ++iteration;
    bool is_ready;
    rcl_ret_t ret = rcl_action_server_is_available(node, client, &is_ready);
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


int ros_action_client_init(struct ros_action_client_t *ros_action_client, struct ros_node_t * ros_node, const rosidl_action_type_support_t * action_type, char* action_name, uint32_t wait_time)
{

    ros_action_client->node = &ros_node->node;
    ros_action_client->wait_time = wait_time;

    rcl_ret_t rc = 0;

    rcl_action_client_options_t action_ops = rcl_action_client_get_default_options();
    
    rc = rcl_action_client_init(
        &ros_action_client->action,
        ros_action_client->node,
        action_type,
        action_name,
        &action_ops);

    if (RCL_RET_OK != rc) 
    {
        panic("[ROS Action client]  Error in rcl_action_client_init %s; error code: %d \n", action_name, rc);
        return -1;
    }
    return rc;
}


int ros_action_client_destroy(struct ros_action_client_t *ros_action_client)
{
    return rcl_action_client_fini(&ros_action_client->action, ros_action_client->node);
}


typedef struct 
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} _dummy_sendgoal_response;

int ros_action_client_goal_try_take(struct ros_action_client_t *ros_action_client,  uint32_t * accept)
{
    rcl_ret_t rc;

    _dummy_sendgoal_response sendgoal_resp;
    
    rc = rcl_action_take_goal_response(
        &ros_action_client->action,
        &ros_action_client->request_id,
        &sendgoal_resp);

    
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_ACTION_CLIENT_TAKE_FAILED)
        {
            debug("[ROS Action client] Error number: %d\n", rc);
            return -1;
        }
        else
        {
            //debug("[ROS Service client] Return code : %d\n", rc);
            return -2;
        }        
    }
    else
    {
        /* code */
        if(sendgoal_resp.accepted == 1)
        {
            *accept = 1;
        }
        else
        {
            *accept = 0;
        }
        

    }
    

    return 0;
}


int ros_action_client_goal_take(struct ros_action_client_t *ros_action_client, uint32_t * accept)
{
    while(ros_action_client_goal_try_take(ros_action_client, accept) != 0)
        usleep(ros_action_client->wait_time);

    return 0;
}




int ros_action_client_goal_send(struct ros_action_client_t *ros_action_client, void * req)
{
    rcl_ret_t rc = 0;
  


    if(wait_for_server_to_be_available( ros_action_client->node, &ros_action_client->action, 100, 1000) == false)
    {
         debug("[ROS Action client] Server NOT available! \n");
         return -1;
    }
    else 
    {
        debug("[ROS Action client] Server available! \n");
    }


    rc = rcl_action_send_goal_request(
        &ros_action_client->action,
        req,
        &ros_action_client->seq_num);

    if (RCL_RET_OK != rc) 
    {
        panic("[ROS Action client]  Error in rcl_action_send_goal_request: error code: %d \n", rc);
        return -1;
    }


    return rc;
    
}





int ros_action_client_feedback_try_take(struct ros_action_client_t *ros_action_client, void*  feedback_msg)
{
    rcl_ret_t rc = 0;

    rc = rcl_action_take_feedback(
            &ros_action_client->action,
            feedback_msg);

    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SERVICE_TAKE_FAILED)
        {
            debug("[ROS Action client] Error number: %d\n", rc);
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


int ros_action_client_feedback_take(struct ros_action_client_t *ros_action_client, void*  feedback_msg)
{
    while(ros_action_client_feedback_try_take(ros_action_client, feedback_msg) != 0)
        usleep(ros_action_client->wait_time);

    return 0;
}



typedef struct
{
  unique_identifier_msgs__msg__UUID goal_id;
} _dummy_GetResult_Request; 
    
int ros_action_client_result_request(struct ros_action_client_t *ros_action_client)
{
    rcl_ret_t rc = 0;

    _dummy_GetResult_Request res_request;

    rc = rcl_action_send_result_request(
        &ros_action_client->action,
        &res_request,
        &ros_action_client->seq_num);

    return rc;
}

int ros_action_client_result_try_take(struct ros_action_client_t *ros_action_client, void * res_result)
{
    rcl_ret_t rc = 0;

    

    rc = rcl_action_take_result_response(
        &ros_action_client->action,
        &ros_action_client->request_id,
        res_result );

    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SERVICE_TAKE_FAILED)
        {
            debug("[ROS Action client] ros_action_client_result_try_take : Error number: %d\n", rc);
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


int ros_action_client_result_take(struct ros_action_client_t *ros_action_client, void * res_result)
{
    while(ros_action_client_result_try_take(ros_action_client, res_result) != 0)
        usleep(ros_action_client->wait_time);

    return 0;
}





