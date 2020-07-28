/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSACTIONCLIENT_H
#define ROSACTIONCLIENT_H

#include <rcl/node.h>
#include <rcl_action/action_client.h>
#include <stdint.h>
#include "ros.h"
#include "rosidl_generator_c/action_type_support_struct.h"

/*
 * Structure representing a mbox
 */
struct ros_action_client_t 
{
    rcl_node_t*             node;
    rcl_action_client_t     action;
    rmw_request_id_t        request_id;   
    uint32_t                wait_time;
    int64_t                 seq_num;
};


#define ROS_ACTIONCLIENT_GOAL_REJECTED       0
#define ROS_ACTIONCLIENT_GOAL_ACCEPTED       1

extern int ros_action_client_init(struct ros_action_client_t *ros_action_client, struct ros_node_t * ros_node, const rosidl_action_type_support_t * action_type, char* action_name, uint32_t wait_time);

extern int ros_action_client_destroy(struct ros_action_client_t *ros_action_client);

extern int ros_action_client_goal_send(struct ros_action_client_t *ros_action_client, void * req);

extern int ros_action_client_goal_take(struct ros_action_client_t *ros_action_client,  uint32_t * accept);

extern int ros_action_client_goal_try_take(struct ros_action_client_t *ros_action_client,  uint32_t * accept);

extern int ros_action_client_feedback_take(struct ros_action_client_t *ros_action_client, void*  feedback_msg);

extern int ros_action_client_feedback_try_take(struct ros_action_client_t *ros_action_client, void*  feedback_msg);

extern int ros_action_client_result_request(struct ros_action_client_t *ros_action_client);

extern int ros_action_client_result_take(struct ros_action_client_t *ros_action_client, void * goal_res);

extern int ros_action_client_result_try_take(struct ros_action_client_t *ros_action_client, void * goal_res);




#endif /* ROSACTIONCLIENT_H */
