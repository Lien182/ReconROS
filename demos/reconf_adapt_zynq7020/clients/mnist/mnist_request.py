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
import cv2
from cv_bridge import CvBridge

from mnist_msgs.srv import Mnist

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('mnist_service_client')
        self.cli = self.create_client(Mnist, 'Mnist')
        while not self.cli.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Mnist.Request()
        self.bridge = CvBridge()

    def send_request(self):
        img_rows, img_cols = 28, 28
        (x_train, y_train), (x_test, self.y_test) = mnist.load_data()

        if K.image_data_format() == 'channels_first':
            self.x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
        else:
            self.x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols , 1)

        #if K.image_data_format() == 'channels_first':
        #    self.x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
        #else:
        #    self.x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)

        self.req.rawdigit = self.bridge.cv2_to_imgmsg(np.array(self.x_test[86]), "mono8")
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('Step: %d' % (self.req.rawdigit.step))


            
def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Prediction: %d' % (response.digit))
            break

                    
    cv2_img = minimal_client.bridge.imgmsg_to_cv2(minimal_client.req.rawdigit, "mono8")
    cv2.imshow('image',cv2_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
