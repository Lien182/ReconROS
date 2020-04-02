/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSSUB_H
#define ROSSUB_H

#include <std_msgs/msg/string.h>

#include <rcl/node.h>
#include <rcl/subscription.h>
#include <stdint.h>
#include "ros.h"

/*
 * Structure representing a mbox
 */
struct ros_subscriber_t {
    rcl_node_t*             node;
    rcl_subscription_t      sub;
    uint32_t                max_msg_size;
    std_msgs__msg__String   sub_msg;
};

/*
 * Initializes the mbox. You must call this method before you
 * can use the mbox.
 *
 *  mb   - pointer to the mbox
 *  size - size of the mbox in 32bit-words
 */
extern int ros_subscriber_init(struct ros_subscriber_t *ros_sub, struct ros_node_t * ros_node, char* topic, uint32_t max_msg_size);

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
extern int ros_subscriber_destroy(struct ros_subscriber_t *ros_sub);

/*
 * Puts a single word into the mbox and blocks if it is full.
 *
 *   mb  - pointer to the mbox
 *   msg - message to put into the mbox
 */
extern int ros_subscriber_try_take(struct ros_subscriber_t *ros_sub, uint8_t * msg);


#endif /* ROSSUB_H */
