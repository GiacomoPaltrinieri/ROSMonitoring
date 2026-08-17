#!/usr/bin/env python3
"""Publish the latest measured joint state at a monitor-friendly rate."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSampler(Node):
    def __init__(self):
        super().__init__("joint_state_sampler")
        self.latest = None
        self.last_arm_positions = None
        self.publisher = self.create_publisher(
            JointState, "/a200_0000/monitor_joint_states", 1
        )
        self.create_subscription(
            JointState, "/a200_0000/platform/joint_states", self.store, 1
        )
        self.create_timer(0.2, self.publish_latest)

    def store(self, message: JointState) -> None:
        self.latest = message

    def publish_latest(self) -> None:
        if self.latest is None:
            return
        positions = dict(zip(self.latest.name, self.latest.position))
        arm_positions = tuple(
            positions.get(f"arm_0_joint_{index}") for index in range(1, 7)
        )
        if None in arm_positions:
            return
        if self.last_arm_positions is None or any(
            abs(current - previous) > 0.01
            for current, previous in zip(arm_positions, self.last_arm_positions)
        ):
            self.publisher.publish(self.latest)
            self.last_arm_positions = arm_positions


def main() -> None:
    rclpy.init()
    node = JointStateSampler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
