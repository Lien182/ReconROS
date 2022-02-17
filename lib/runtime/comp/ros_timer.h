/*
 * Copyright 2021 Christian Lienen <christian.lienen@upb.de>
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

    float       fInterval;   // Interval in ms
    uint64_t    u64Interval; // Interval in TimerTicks

    uint64_t    u64NextEventTime; 
    uint64_t    u64LastEventTime;
};


int ros_timer_init(struct ros_timer_t *ros_timer, struct ros_node_t * ros_node, float interval);

int ros_timer_destroy(struct ros_timer_t *ros_timer);

int ros_timer_is_ready(struct ros_timer_t * ros_timer, uint32_t * timer_value);
#endif /* ROSTMR_H */
