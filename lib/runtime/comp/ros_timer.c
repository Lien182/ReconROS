/*
 * Copyright 2021 Christian Lienen <christian.lienen@upb.de>
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>


#include "ros_timer.h"

#include "../arch/a9_timer.h"


int ros_timer_init(struct ros_timer_t *ros_timer, struct ros_node_t * ros_node, float interval) // intervall in ms
{
    ros_timer->fInterval = interval;
    ros_timer->u64Interval = a9timer_msto(interval);
    ros_timer->u64LastEventTime = a9timer_get();
    ros_timer->u64NextEventTime = ros_timer->u64LastEventTime + ros_timer->u64Interval; 
    printf("[ROS TMR] fInterval=%f, u64Interval= %lld, u64NextInterval= %lld \n", interval, ros_timer->u64Interval, ros_timer->u64NextEventTime);
    return 0;
}

int ros_timer_destroy(struct ros_timer_t *ros_timer)
{
    return 0;
}


int ros_timer_is_ready(struct ros_timer_t * ros_timer, uint32_t * timer_value)
{
    uint64_t u64TimeNow = a9timer_get();
    //  printf("Timer Value: %lld \n", u64TimeNow);
    if(u64TimeNow >= ros_timer->u64NextEventTime)
    {
        ros_timer->u64NextEventTime += ros_timer->u64Interval; 
        ros_timer->u64LastEventTime = u64TimeNow;
        *timer_value = (uint32_t)u64TimeNow;
        return 0;
    }

    return 1;
}