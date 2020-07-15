/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROS_H
#define ROS_H

#include <rcl/context.h>
#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rcl/init_options.h>

/*
 * Structure representing a ros node
 */
struct ros_node_t {
    uint32_t            node_nr;
    rcl_context_t       context;
    rcl_init_options_t  init_options;
    rcl_node_t          node;
    rcl_allocator_t     allocator;
};

/*
 * Initiali;zes the ros node. You must call this method before you
 * can use the ros node .
 *
 *  ros_node   - pointer to the ros_node
 * 
 */
extern int ros_node_init(struct ros_node_t *ros_node, char * name);

/*
 * Frees all used memory of the ros_node.
 *
 *   ros_node - pointer to the rosnode 
 */
extern int ros_node_destroy(struct ros_node_t *ros_node);



#endif /* ROS_H */
