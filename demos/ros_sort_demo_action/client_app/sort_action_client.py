import random
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sorter_msgs.action import Sort

class SortActionClient(Node):

    def __init__(self):
        super().__init__('sort_action_client')
        self._action_client = ActionClient(self, Sort, 'sorter')

    def send_goal(self, order):
        goal_msg = Sort.Goal()
        goal_msg.unsorted = random.sample(range(1, 100000), 2048)

        self._action_client.wait_for_server()

        #self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0} percent done'.format(feedback.percent))

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.sorted.tolist() == sorted(result.sorted):
            self.get_logger().info('Data is sorted!')
        else:
            self.get_logger().info('Data is NOT sorted!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = SortActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
