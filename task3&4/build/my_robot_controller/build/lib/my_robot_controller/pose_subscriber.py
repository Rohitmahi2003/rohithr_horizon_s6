#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):  # Fixed class name

    def __init__(self):
        super().__init__("pose_subscriber")  # Fixed node name
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
    
    def pose_callback(self, msg: Pose):
        self.get_logger().info("("+ str(msg.x)+ " ," + str(msg.y)+")")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()  # Fixed class name usage
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()
