/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSSERVICESERVER_H
#define ROSSERVICESERVER_H

#include <rcl/node.h>
#include <rcl/service.h>
#include <stdint.h>
#include "ros.h"

/*
 * Structure representing a mbox
 */
struct ros_service_server_t 
{
    rcl_node_t*             node;
    rcl_service_t           service;
    rmw_request_id_t        request_id;   
    uint32_t                wait_time;    
};


extern int ros_service_server_init(struct ros_service_server_t *ros_service_server, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* topic, uint32_t wait_time);

extern int ros_service_server_destroy(struct ros_service_server_t *ros_service_server);

extern int ros_service_server_request_try_take(struct ros_service_server_t *ros_service_server, void * req);

extern int ros_service_server_request_take(struct ros_service_server_t *ros_service_server, void * req);

extern int ros_service_server_response_send(struct ros_service_server_t *ros_service_server, void * res);

#endif /* ROSSERVICESERVER_H */
