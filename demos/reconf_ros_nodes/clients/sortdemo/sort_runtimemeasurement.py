import rclpy
from rclpy.node import Node
from rclpy.client import Client
import time
import struct
import random
import os
import numpy as np
from sorter_msgs.srv import Sort




class SortClient(Node):

    def __init__(self):
        super().__init__('sorter_client')
        self.cli = self.create_client(Sort, 'sorter')
        while not self.cli.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Sort.Request()

    def send_request(self):
        self.req.unsorted =  random.sample(range(1, 100000), 2048)
        self.future = self.cli.call_async(self.req)


            
def main(args=None):
    rclpy.init(args=args)

    sort_client = SortClient()
    sort_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(sort_client)
        if sort_client.future.done():
            try:
                response = sort_client.future.result()
            except Exception as e:
                sort_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                
                if response.sorted.tolist() == sorted(response.sorted):
                    sort_client.get_logger().info('Data is sorted!')
                else:
                    sort_client.get_logger().info('Data is NOT sorted!')
            break

                    
    sort_client.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()

