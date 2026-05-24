#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from mission_interface.msg import TargetState
from mission_interface.msg import SensorState
from mission_interface.msg import ReferenceState


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.current_state_sub = self.create_subscription(
            SensorState,
            "/measured_state",
            self.current_state_listener,
            10
        )

        self.target_state_sub = self.create_subscription(
            TargetState,
            "/target_state",
            self.target_state_listener,
            10
        )

        self.reference_state_pub = self.create_publisher(
            ReferenceState,
            "/reference_state",
            10
        )

        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]

        self.target_position = [0.0, 0.0, 0.0]
        self.target_velocity = [0.0, 0.0, 0.0]

        self.planner_timer = self.create_timer(
            0.1,
            self.planner_callback
        )


    def current_state_listener(self, msg):
        self.current_position = msg.measured_position
        self.current_velocity = msg.measured_velocity

    def target_state_listener(self, msg):
        self.target_position = msg.target_position
        self.target_velocity = msg.target_velocity

    def planner_callback(self):
        current_position_np = np.array(self.current_position)
        target_position_np = np.array(self.target_position)

        direction = target_position_np - current_position_np
        distance = np.linalg.norm(direction)

        max_step_size = 0.1
        desired_speed = 1.0

        if distance > 0.001:
            direction_unit = direction / distance

            step_size = min(max_step_size, distance)          

            reference_position = current_position_np + direction_unit * step_size

            if distance < max_step_size:
                reference_velocity = direction_unit * (desired_speed * (distance / max_step_size))
            else:
                reference_velocity = direction_unit * desired_speed
        else:

            reference_position = target_position_np
            reference_velocity = np.array([0.0, 0.0, 0.0])

        trajectory_msg = ReferenceState()

        trajectory_msg.position_trajectory = reference_position.tolist()
        trajectory_msg.velocity_trajectory = reference_velocity.tolist()

        self.reference_state_pub.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()