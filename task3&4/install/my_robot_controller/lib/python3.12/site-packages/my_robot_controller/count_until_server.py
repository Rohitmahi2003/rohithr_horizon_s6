#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_controller.action import CountUntil  # Import the action interface
import time

class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        self.get_logger().info("Count Until Action Server is running...")

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: Count up to {goal_handle.request.target_number}")

        feedback_msg = CountUntil.Feedback()
        result = CountUntil.Result()
        
        count = 0
        while count < goal_handle.request.target_number:
            time.sleep(1)  # Simulate a time-consuming task
            count += 1
            feedback_msg.current_number = count
            self.get_logger().info(f"Feedback: {count}")
            goal_handle.publish_feedback(feedback_msg)

        result.reached_number = count
        goal_handle.succeed()
        self.get_logger().info(f"Goal completed: Reached {count}")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
