/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSACTIONSERVER_H
#define ROSACTIONSERVER_H

#include <rcl/node.h>
#include <rcl_action/action_server.h>
#include <stdint.h>
#include "ros.h"
#include "rosidl_generator_c/action_type_support_struct.h"

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
};


extern int ros_action_server_init(struct ros_action_server_t *ros_action_server, struct ros_node_t * ros_node, const rosidl_action_type_support_t * action_type, char* action_name, uint32_t wait_time);

extern int ros_action_server_destroy(struct ros_action_server_t *ros_action_server);

extern int ros_action_server_try_take_request(struct ros_action_server_t *ros_action_server_t, void * req);

extern int ros_action_server_try_take_goal_request(struct ros_action_server_t *ros_action_server, void * req);

extern int ros_action_server_take_goal_request(struct ros_action_server_t *ros_action_server, void * req);

extern int ros_action_server_take_decide_request(struct ros_action_server_t *ros_action_server, void * goal_res, uint32_t accept);

extern int ros_action_server_publish_feedback(struct ros_action_server_t *ros_action_server, void*  feedback_msg);

extern int ros_action_server_try_take_result_request(struct ros_action_server_t *ros_action_server, void * goal_req);

extern int ros_action_server_take_result_request(struct ros_action_server_t *ros_action_server, void * goal_req);

extern int ros_action_server_send_result_response(struct ros_action_server_t *ros_action_server, void * goal_res);


#endif /* ROSACTIONSERVER_H */
