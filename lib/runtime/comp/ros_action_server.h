/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSACTIONSERVER_H
#define ROSACTIONSERVER_H

#include <rcl/node.h>
#include <rcl_action/action_server.h>
#include <stdint.h>
#include "ros.h"
#include "rosidl_runtime_c/action_type_support_struct.h"

/*
 * Structure representing a mbox
 */
struct ros_action_server_t 
{
    rcl_node_t*             node;
    rcl_action_server_t     action;
    rmw_request_id_t        request_id;   
    uint32_t                wait_time;
    rcl_clock_t             clock;

    uint32_t                shutdown;  
    uint32_t                fsm;  
};


#define ROS_ACTIONSERVER_GOAL_REJECT       0
#define ROS_ACTIONSERVER_GOAL_ACCEPT       1

extern int ros_action_server_init(struct ros_action_server_t *ros_action_server, struct ros_node_t * ros_node, const rosidl_action_type_support_t * action_type, char* action_name, uint32_t wait_time);

extern int ros_action_server_destroy(struct ros_action_server_t *ros_action_server);

extern int ros_action_server_goal_try_take(struct ros_action_server_t *ros_action_server, void * req);

extern int ros_action_server_goal_take(struct ros_action_server_t *ros_action_server, void * req);

extern int ros_action_server_goal_decide(struct ros_action_server_t *ros_action_server,  uint32_t accept);

extern int ros_action_server_feedback(struct ros_action_server_t *ros_action_server, void*  feedback_msg);

extern int ros_action_server_result_try_take(struct ros_action_server_t *ros_action_server);

extern int ros_action_server_result_take(struct ros_action_server_t *ros_action_server);

extern int ros_action_server_result_send(struct ros_action_server_t *ros_action_server, void * goal_res);


#endif /* ROSACTIONSERVER_H */
