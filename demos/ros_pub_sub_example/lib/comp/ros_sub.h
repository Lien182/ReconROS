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
    rcl_node_t*                 node;
    rcl_subscription_t          sub;
    uint32_t                    max_msg_size;
    rcl_serialized_message_t    sub_msg;
    uint32_t                    wait_time;
};

/*
 * Initializes the mbox. You must call this method before you
 * can use the mbox.
 *
 *  mb   - pointer to the mbox
 *  size - size of the mbox in 32bit-words
 */
extern int ros_subscriber_init(struct ros_subscriber_t *ros_sub, struct ros_node_t * ros_node, char* topic, uint32_t max_msg_size, uint32_t wait_time);

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
extern int ros_subscriber_destroy(struct ros_subscriber_t *ros_sub);


/*
 * receive a message from the given topic.
 *
 *   mb  - pointer to the subscriber instance
 *   msg - message pointer to the received message
 *   len - length of the received message
 * 
 *   
 */
extern int ros_subscriber_take(struct ros_subscriber_t *ros_sub, uint8_t ** msg, uint32_t * len);

/*
 * Try to receive a message from the given topic.
 *
 *   mb  - pointer to the subscriber instance
 *   msg - message pointer to the received message
 *   len - length of the received message
 * 
 *   return 0 if data is available
 */
extern int ros_subscriber_try_take(struct ros_subscriber_t *ros_sub, uint8_t ** msg, uint32_t * len);


#endif /* ROSSUB_H */
