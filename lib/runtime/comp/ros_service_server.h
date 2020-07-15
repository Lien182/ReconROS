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
struct ros_service_server_t {
    rcl_node_t*             node;
    rcl_service_t           service;
    rmw_request_id_t        request_id;
   
    uint32_t                wait_time;
    
};

/*
 * Initializes the mbox. You must call this method before you
 * can use the mbox.
 *
 *  mb   - pointer to the mbox
 *  size - size of the mbox in 32bit-words
 */
extern int ros_service_server_init(struct ros_service_server_t *ros_service_server, struct ros_node_t * ros_node, const rosidl_service_type_support_t * srv_type, char* topic, uint32_t wait_time);

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
extern int ros_service_server_destroy(struct ros_service_server_t *ros_service_server);



extern int ros_service_server_try_take_request(struct ros_service_server_t *ros_service_server, void * req);

extern int ros_service_server_take_request(struct ros_service_server_t *ros_service_server, void * req);

extern int ros_service_server_send_response(struct ros_service_server_t *ros_service_server, void * res);

#endif /* ROSSERVICESERVER_H */
