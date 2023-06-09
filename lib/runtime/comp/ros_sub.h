/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSSUB_H
#define ROSSUB_H

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
    void*                       message;
    uint32_t                    wait_time;
    rmw_subscription_allocation_t alloc;
    char*                       topic; 
    uint32_t                    bFilterSet;
    char                        filter[24];
};

/*
 * Initializes the mbox. You must call this method before you
 * can use the mbox.
 *
 *  ros_sub   - pointer to the subscriber
 *  ros_node - ros node
 */
extern int ros_subscriber_init(struct ros_subscriber_t *ros_sub, struct ros_node_t * ros_node, const rosidl_message_type_support_t * msg_type, char* topic, uint32_t wait_time);

/*
 * Frees all used memory of the mbox.
 *
 *   ros_sub - pointer to the subscriber
 */
extern int ros_subscriber_destroy(struct ros_subscriber_t *ros_sub);


/*
 * receive a message from the given topic.
 *
 *   ros_sub  - pointer to the subscriber instance
 *   msg - message pointer to the received message
 *   len - length of the received message
 * 
 *   
 */
extern int ros_subscriber_message_take(struct ros_subscriber_t *ros_sub, void * msg);


/*
 * Try to receive a message from the given topic.
 *
 *   ros_sub  - pointer to the subscriber instance
 *   msg - message pointer to the received message
 *   len - length of the received message
 * 
 *   return 0 if data is available
 */
extern int ros_subscriber_message_try_take(struct ros_subscriber_t *ros_sub, void * msg);


/*
 * sets the filter to prevent messages from a certain publisher
 *
 *   ros_sub  - pointer to the subscriber instance
 *   filter - publisher gid to avoid subscriptions from
 * 
 *   return void
 */
extern void ros_subscriber_set_publishfilter(struct ros_subscriber_t *ros_sub, uint8_t * filter);

#endif /* ROSSUB_H */
