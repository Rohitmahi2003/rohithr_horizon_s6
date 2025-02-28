#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_controller.action import CountUntil  # Import the action interface

class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_client')
        self._action_client = ActionClient(self, CountUntil, 'count_until')

    def send_goal(self, target_number):
        goal_msg = CountUntil.Goal()
        goal_msg.target_number = target_number

        self.get_logger().info(f"Sending goal: Count up to {target_number}")
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.current_number}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Final Result: Reached {result.reached_number}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionClient()
    node.send_goal(5)  # Send a goal to count up to 5
    rclpy.spin(node)

if __name__ == "__main__":
    main()
