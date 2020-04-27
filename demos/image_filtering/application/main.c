#include <stdio.h>

#include "../lib/comp/ros.h"
#include "../lib/comp/ros_pub.h"
#include "../lib/comp/ros_sub.h"

#include <sensor_msgs/msg/image.h>

int main(int argc, char ** argv)
{
    struct ros_node_t ros_node;;
    struct ros_subscriber_t ros_sub;

/*Define area here - start*/
    sensor_msgs__msg__Image * msg = sensor_msgs__msg__Image__create();
    const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image);;
/*Define area here - end*/

    if(ros_node_init(&ros_node, "image_example_node"))
    {
        return -1;
    }   

    printf("Node ist da! \n");

    if(ros_subscriber_init(&ros_sub, &ros_node, my_type_support, "/image",  10))
    {
        return -2;
    }


    for(int i = 0; i < 10; i++)
    {
        printf("Probiere! \n");
        ros_subscriber_message_take(&ros_sub, msg);
        printf("Received data: pointer= %x, height = %d \n", msg, msg->height);


    }


    ros_subscriber_destroy(&ros_sub);
    ros_node_destroy(&ros_node);


    return 0;
}