#include <stdio.h>

#include "../lib/comp/ros.h"
#include "../lib/comp/ros_pub.h"
#include "../lib/comp/ros_sub.h"

#include <sensor_msgs/msg/image.h>

int main(int argc, char ** argv)
{
    struct ros_node_t       ros_node;
    struct ros_subscriber_t ros_sub;
    struct ros_publisher_t  ros_pub;

    uint32_t index;

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
        ros_node_destroy(&ros_node);
        return -2;
    }

    if(ros_publisher_init(&ros_pub, &ros_node, my_type_support, "test"))
    {
        
        ros_subscriber_destroy(&ros_sub);
        ros_node_destroy(&ros_node);

        return -3;
    }

    for(int i = 0; i < 100000; i++)
    {
        printf("Probiere! \n");
        ros_subscriber_message_take(&ros_sub, msg);
        printf("Received data: pointer= %x, rawdatasize = %d, height = %d, %d - iteration \n", msg, msg->data.size, msg->height, i);

        for(int ll = 0; ll < msg->height; ll++)
        {
            for(int kk = 0; kk < msg->width; kk++)
            {
                index = ( ll * msg->width + kk )*3;
                msg->data.data[index] = ((uint16_t)msg->data.data[index] + (uint16_t)msg->data.data[index+ 1 ] + (uint16_t)msg->data.data[index + 2])/3; 
                msg->data.data[index+ 1] = msg->data.data[index];
                msg->data.data[index+ 2] = msg->data.data[index]; 
            }
        }

        ros_publisher_publish(&ros_pub, msg);


    }

    ros_publisher_destroy(&ros_pub);
    ros_subscriber_destroy(&ros_sub);
    ros_node_destroy(&ros_node);


    return 0;
}