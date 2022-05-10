import rclpy
from rclpy.node import Node

import time
import struct
import random

import os

import numpy as np

from cv_bridge import CvBridge
from std_msgs.msg import UInt32MultiArray
import cv2
from cv_bridge import CvBridge


class ImagePubSub(Node):

    def __init__(self):
        super().__init__('Image')
        
        self.publisher_ = self.create_publisher(UInt32MultiArray, '/unsorted', 10)
        

        self.tstart = 0
        self.tend = 0       
        self.subscription = self.create_subscription(UInt32MultiArray,'/sorted',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = UInt32MultiArray()
        self.msg.data = random.sample(range(1, 100000), 2048)
        self.cnt = 0


    def listener_callback(self, msg):
        #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
        self.tend = time.time()
        print('%f' % (self.tend - self.tstart))
        self.cnt += 1
        if self.cnt == 1000:
            self.timer.cancel()
            self.destroy_node()
            exit()
        

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.tstart = time.time()
            
def main(args=None):
    rclpy.init(args=args)

    pubsub = ImagePubSub()
    rclpy.spin(pubsub)




if __name__ == '__main__':
    main()
