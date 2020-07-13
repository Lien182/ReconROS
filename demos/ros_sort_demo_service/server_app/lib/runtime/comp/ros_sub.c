/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/image.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_sub.h"
#include "rmw/types.h"

#include "../utils.h"

int ros_subscriber_init(struct ros_subscriber_t *ros_sub, struct ros_node_t * ros_node, const rosidl_message_type_support_t * msg_type, char* topic, uint32_t wait_time)
{
    rcl_ret_t rc;
    // create subscription
    ros_sub->sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();

    //rcl_allocator_t allocator = rcl_get_default_allocator();
    //rmw_init_subscription_allocation(&ros_sub->alloc, msg_type, bounds);


    rc = rcl_subscription_init(
        &ros_sub->sub,
        &ros_node->node,
        msg_type,
        topic,
        &my_subscription_options);  


    if (rc != RCL_RET_OK) {
        panic("[ROS Subscriber] Failed to create subscriber: %s\n", topic);
        return -2;
    } 
    else 
    {
        debug("[ROS Subscriber] Created subscriber: %s\n", topic);
    } 

    ros_sub->wait_time = wait_time;

    
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

int ros_subscriber_message_try_take(struct ros_subscriber_t *ros_sub, void * msg)
{
    rcl_ret_t rc;
    rmw_message_info_t messageInfo;




    
    rc = rcl_take(&ros_sub->sub, msg,  &messageInfo,NULL );
    
    if(rc != RCL_RET_OK)
    {
        if(rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            debug("[ROS Subscriber] Error number: %d\n", rc);
            return -1;
        }
        else
        {
            //debug("[ROS Subscriber] Return code : %d\n", rc);
            return 1;
        }        
    }

    return 0;
}


int ros_subscriber_message_take(struct ros_subscriber_t *ros_sub, void * msg)
{
    while(ros_subscriber_message_try_take(ros_sub, msg) != 0)
        usleep(ros_sub->wait_time);

    return 0;
}