import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import random
import numpy as np
import matplotlib.pyplot as plt
import time
import struct
import random

from tensorflow import keras
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras import backend as K
from keras.models import model_from_json
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32
import cv2
from cv_bridge import CvBridge



class ImagePubSub(Node):

    def __init__(self):
        super().__init__('Image')

        self.bridge = CvBridge()
        
        self.publisher_ = self.create_publisher(Image, '/image_classification', 10)
        
        self.img_rows, self.img_cols = 28, 28
        (self.x_train, self.y_train), (self.x_test, self.y_test) = mnist.load_data()
        
        self.tstart = 0
        self.tend = 0       
        self.subscription = self.create_subscription(UInt32,'/class',self.listener_callback,  10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
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
        if K.image_data_format() == 'channels_first':
            self.x_test = self.x_test.reshape(self.x_test.shape[0], 1, self.img_rows, self.img_cols)
        else:
            self.x_test = self.x_test.reshape(self.x_test.shape[0], self.img_rows, self.img_cols , 1)

        self.msg = self.bridge.cv2_to_imgmsg(np.array(self.x_test[random.randint(0,100)]), "mono8")

        self.publisher_.publish(self.msg)
        self.tstart = time.time()
            
def main(args=None):
    rclpy.init(args=args)

    pubsub = ImagePubSub()
    rclpy.spin(pubsub)


if __name__ == '__main__':
    main()