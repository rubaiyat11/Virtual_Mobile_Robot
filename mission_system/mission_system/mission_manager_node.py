#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mission_interface.action import GoToTarget


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(Node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()