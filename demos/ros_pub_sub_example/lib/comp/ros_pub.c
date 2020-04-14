/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>
#include <rosidl_generator_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "ros_pub.h"

#include "../utils.h"

int ros_publisher_init(struct ros_publisher_t *ros_pub, struct ros_node_t * ros_node , char* topic_name, uint32_t max_msg_size)
{
  rcl_node_t* node = &ros_node->node;

  ros_pub->max_msg_size = max_msg_size;
  
  rcl_ret_t rc = 0;

  const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  rc = rcl_publisher_init(
      &ros_pub->rcl_pub,
      node,
      my_type_support,
      topic_name,
      &pub_options);
  
  if (RCL_RET_OK != rc) {
    panic("[ROS Publisher] Error in rcl_publisher_init %s\n", topic_name);
    return -1;
  }

  return 0;
}

int ros_publisher_destroy(struct ros_publisher_t *ros_pub)
{
  return rcl_publisher_fini(&ros_pub->rcl_pub, ros_pub->node);
}

int ros_publisher_publish(struct ros_publisher_t *ros_pub, uint8_t * msg, uint32_t msg_size)
{
  rcl_ret_t rc;

  rcl_serialized_message_t seri_msg;

  seri_msg.buffer = msg;
  seri_msg.buffer_length = msg_size;
  seri_msg.buffer_capacity = ros_pub->max_msg_size;
  seri_msg.allocator = rcl_get_default_allocator();
  
  rc = rcl_publish_serialized_message(&ros_pub->rcl_pub, &seri_msg, NULL);
  if (rc == RCL_RET_OK) {
    debug("[ROS Publisher] Published message!\n");
  } else {
    debug("[ROS Publisher] Error publishing message!\n");
  }
  return 0;
}
