import rclpy
from rclpy.node import Node

import time
import struct
import random

import os

import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImagePubSub(Node):

    def __init__(self):
        super().__init__('Image')
        
        self.publisher_ = self.create_publisher(Image, 'inputtopic', 10)
        self.cv_image = cv2.imread('image.jpg') ### an RGB image 
        self.bridge = CvBridge()
        self.tstart = 0
        self.tend = 0       
        self.subscription = self.create_subscription(Image,'/topic/gateway',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8")
        self.cnt = 0


    def listener_callback(self, msg):
        #cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
        self.tend = time.time()
        print('%f' % (self.tend - self.tstart))
        self.cnt += 1
        exit()       
        
        

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        print("Data published!")
        self.tstart = time.time()
        self.timer.cancel()
            
def main(args=None):
    rclpy.init(args=args)

    pubsub = ImagePubSub()
    rclpy.spin(pubsub)




if __name__ == '__main__':
    main()
