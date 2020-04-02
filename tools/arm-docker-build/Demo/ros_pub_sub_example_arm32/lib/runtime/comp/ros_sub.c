/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>
#include <std_msgs/msg/string.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>


#include "ros_sub.h"
#include "../utils.h"
#include "rmw/types.h"


int ros_subscriber_init(struct ros_subscriber_t *ros_sub, struct ros_node_t * ros_node, char* topic, uint32_t max_msg_size)
{
    rcl_ret_t rc;
    // create subscription
    ros_sub->sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();

    const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    std_msgs__msg__String__init(&ros_sub->sub_msg);

    my_subscription_options.qos.depth = 10;


    rc = rcl_subscription_init(
        &ros_sub->sub,
        &ros_node->node,
        my_type_support,
        topic,
        &my_subscription_options);  

    if (rc != RCL_RET_OK) {
        debug("Failed to create subscriber %s.\n", topic);
        return -1;
    } 
    else 
    {
        debug("Created subscriber %s:\n", topic);
    } 

    return 0;
}

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
int ros_subscriber_destroy(struct ros_subscriber_t *ros_sub)
{
    return rcl_subscription_fini(&ros_sub->sub, ros_sub->node);
}

/*
 * Puts a single word into the mbox and blocks if it is full.
 *
 *   mb  - pointer to the mbox
 *   msg - message to put into the mbox
 */
int ros_subscriber_try_take(struct ros_subscriber_t *ros_sub, uint8_t * msg)
{
    rcl_ret_t rc;
    rmw_message_info_t messageInfo;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    
    //rmw_subscription_allocation_t allocator;
    //rmw_init_subscription_allocation(&allocator, ts, bounds);

    rc = rcl_take(&ros_sub->sub, &ros_sub->sub_msg, &messageInfo, NULL);
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            debug("Error number: %d\n", rc);
            return -1;
        }
        else
        {
            printf("Return code : %d\n", rc);
            return 1;
        }        
    }
    memcpy(msg, ros_sub->sub_msg.data.data, ros_sub->sub_msg.data.size);
    return 0;
}