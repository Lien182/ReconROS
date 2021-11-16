/*
 * Copyright 2020 Christian Lienen <christian.lienen@upb.de>
 */

#ifndef ROSTMR_H
#define ROSTMR_H

#include <rcl/node.h>
#include <stdint.h>
#include "ros.h"

/*
 * Structure representing a mbox
 */
struct ros_timer_t {

};


int ros_timer_init(struct ros_timer_t *ros_timer, struct ros_node_t * ros_node, uint32_t interval);


int ros_timer_destroy(struct ros_timer_t *ros_timer);


int ros_timer_wait(struct ros_timer_t *ros_timer);


#endif /* ROSSUB_H */
