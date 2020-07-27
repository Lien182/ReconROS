/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSSERVICECLIENT_H
#define ROSSERVICECLIENT_H

#include <rcl/node.h>
#include <rcl/client.h>
#include <stdint.h>
#include "ros.h"

/*
 * Structure representing a mbox
 */
struct ros_service_client_t 
{
    rcl_node_t*             node;
    rcl_client_t           service;
    rmw_request_id_t        request_id;   
    uint32_t                wait_time;    
};


extern int ros_service_client_init(struct ros_service_client_t *ros_service_client, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* topic, uint32_t wait_time);

extern int ros_service_client_destroy(struct ros_service_client_t *ros_service_client);

extern int ros_service_client_send_request(struct ros_service_client_t *ros_service_client, void * req);

extern int ros_service_client_try_take_response(struct ros_service_client_t *ros_service_client, void * res);

extern int ros_service_client_take_response(struct ros_service_client_t *ros_service_client, void * res);


#endif /* ROSSERVICEclient_H */
