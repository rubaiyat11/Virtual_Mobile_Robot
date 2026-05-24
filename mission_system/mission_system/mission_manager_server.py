#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from mission_interface.action import NavigateToTarget
from mission_interface.msg import SensorState
from mission_interface.msg import TargetState


class MissionManagerServer(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.mission_manager_server_ = ActionServer(
            self,
            NavigateToTarget,
            "navigate_to_target",
            execute_callback=self.execute_callback
        )

        self.sensor_state_sub = self.create_subscription(
            SensorState,
            "/measured_state",
            self.listener_callback,
            10
        )

        self.target_pub = self.create_publisher(
            TargetState,
            "/target_state",
            10
        )

        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]

    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_position = goal_handle.request.target_position
        position_tolerance = goal_handle.request.position_tolerance

        target_msg = TargetState()
        target_msg.target_position = target_position
        target_msg.target_velocity = goal_handle.request.target_velocity
        self.target_pub.publish(target_msg)

        feedback_msg = NavigateToTarget.Feedback()

        while rclpy.ok():
        
            rclpy.spin_once(self, timeout_sec=0.1)

            current_position_np = np.array(self.current_position)
            target_position_np = np.array(target_position)

            position_error = np.linalg.norm(target_position_np - current_position_np)
            
            self.get_logger().info(f"Position Error: {position_error:.4f}")
            
            feedback_msg.current_position = self.current_position
            feedback_msg.distance_remaining = position_error
            goal_handle.publish_feedback(feedback_msg)

            if position_error < position_tolerance:
                goal_handle.succeed()
                result = NavigateToTarget.Result()
                result.success = True
                result.message = "Target reached"
                return result

    def listener_callback(self, msg: SensorState):
        self.current_position = list(msg.measured_position)
        self.current_velocity = list(msg.measured_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()