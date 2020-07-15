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

int ros_service_server_init(struct ros_service_server_t *ros_service_server, struct ros_node_t * ros_node, const rosidl_message_type_support_t * msg_type, char* topic, uint32_t wait_time)
{
    return 0;
}

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
int ros_service_server_destroy(struct ros_service_server_t *ros_service_server)
{
    //return rcl_subscription_fini(&ros_sub->sub, ros_sub->node);
}

