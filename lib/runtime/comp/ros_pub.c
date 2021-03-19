/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>
#include <std_msgs/msg/string.h>

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "ros_pub.h"

#include "../utils.h"

int ros_publisher_init(struct ros_publisher_t *ros_pub, struct ros_node_t * ros_node, const rosidl_message_type_support_t * msg_type, char* topic_name)
{
  rcl_ret_t rc = 0;

  ros_pub->rcl_pub= rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();

  rc = rcl_publisher_init(
      &ros_pub->rcl_pub,
      &(ros_node->node),
      msg_type,
      topic_name,
      &pub_options);
  
  if (RCL_RET_OK != rc) 
  {
    panic("[ROS Publisher] Error in rcl_publisher_init %s; error code: %d \n", topic_name, rc);
    return -1;
  }
  else 
  {
    debug("[ROS Publisher] Created publisher: %s\n", topic_name);
  } 

  return 0;
}

int ros_publisher_destroy(struct ros_publisher_t *ros_pub)
{
  return rcl_publisher_fini(&ros_pub->rcl_pub, ros_pub->node);
}

int ros_publisher_publish(struct ros_publisher_t *ros_pub, void * msg)
{
  rcl_ret_t rc;

  rc = rcl_publish(&ros_pub->rcl_pub, msg, NULL);
  if (rc == RCL_RET_OK) {
    debug("[ROS Publisher] Published message!\n");
    return 0;
  } else {
    debug("[ROS Publisher] Error publishing message!\n");
    return rc;
  }
}
