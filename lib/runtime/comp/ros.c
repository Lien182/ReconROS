/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */
 #define _GNU_SOURCE 
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>

#include "ros.h"

#include <rcl/rcl.h>

#include "../utils.h"

static uint32_t node_nr = 0;
static pthread_mutex_t mutexThreadInit = PTHREAD_MUTEX_INITIALIZER;

int ros_node_init(struct ros_node_t *ros_node)
{
  char * node_name;
  ros_node->context = rcl_get_zero_initialized_context();
  ros_node->init_options = rcl_get_zero_initialized_init_options();
  ros_node->allocator = rcl_get_default_allocator();
  rcl_ret_t rc;

  //reset errors
  rcutils_reset_error();

  //Makes this init function thread safe
  pthread_mutex_lock(&mutexThreadInit);

  // create init_options
  rc = rcl_init_options_init(&ros_node->init_options, ros_node->allocator);
  if (rc != RCL_RET_OK) {
    pthread_mutex_unlock(&mutexThreadInit);
    panic("Error rcl_init_options_init.\n");
    return -1;
  }

  
  // create context
  rc = rcl_init(0, 0, &ros_node->init_options, &ros_node->context);
  if (rc != RCL_RET_OK) {
    pthread_mutex_unlock(&mutexThreadInit);
    panic("Error in rcl_init.\n");
    return -1;
  }
  
  // create rcl_node
  ros_node->node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  ros_node->node_nr = node_nr;

  if(asprintf(&node_name, "ReconROS_Node_%d", node_nr++) < 0)
  {
    pthread_mutex_unlock(&mutexThreadInit);
    return -2;
  }
    

  rc = rcl_node_init(&ros_node->node, node_name, "", &ros_node->context, &node_ops);
  if (rc != RCL_RET_OK) {
    free(node_name);
    pthread_mutex_unlock(&mutexThreadInit);
    panic("Error in rcl_node_init\n");
    return -1;
  }

  pthread_mutex_unlock(&mutexThreadInit);

  free(node_name);
  return 0;
}

int ros_node_destroy(struct ros_node_t *ros_node)
{
  int ret = 0;
  ret += rcl_node_fini(&ros_node->node);
  ret += rcl_init_options_fini(&ros_node->init_options);
  return ret;
}