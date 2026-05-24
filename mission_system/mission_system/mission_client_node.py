#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from mission_interface.action import NavigateToTarget


class MissionClientNode(Node):
    def __init__(self):
        super().__init__("mission_client_node")

        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 0.0)

        tx = self.get_parameter("target_x").get_parameter_value().double_value
        ty = self.get_parameter("target_y").get_parameter_value().double_value
        tz = self.get_parameter("target_z").get_parameter_value().double_value

        self.get_logger().info(f"Target initialized via parameters: X={tx}, Y={ty}, Z={tz}")

        self.mission_client_node_ = ActionClient(
            self, NavigateToTarget, "navigate_to_target"
        )

        self.startup_timer = self.create_timer(0.1, self.timer_startup_callback)

    def timer_startup_callback(self):
          self.startup_timer.cancel()

          self.send_goal(
            target_position=[self.tx, self.ty, self.tz],
            target_velocity=[0.0, 0.0, 0.0],
            position_tolerance=1.0,
            velocity_tolerance=1.0
        )

    def send_goal(self, target_position, target_velocity, position_tolerance, velocity_tolerance):
        self.mission_client_node_.wait_for_server()

        goal = NavigateToTarget.Goal()
        goal.target_position = target_position
        goal.target_velocity = target_velocity
        goal.position_tolerance = position_tolerance
        goal.velocity_tolerance = velocity_tolerance

        self.mission_client_node_.send_goal_async(goal, feedback_callback=self.feedback_callback). \
            add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async(). \
                add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal Rejected")

    def goal_result_callback(self, future):
        result = future.result().result

        self.get_logger().info(
            f"Result: {result.message}"
        )

        self.get_logger().info(
            f"Success: {result.success}"
        )
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f"Distance Remaining: {feedback.distance_remaining}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MissionClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()