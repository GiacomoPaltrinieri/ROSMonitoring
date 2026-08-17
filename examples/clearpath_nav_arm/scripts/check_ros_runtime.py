#!/usr/bin/env python3
"""Fail fast when the case-study ROS middleware cannot create action clients."""

import os

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

import rclpy
from nav2_msgs.action import NavigateToPose


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("nav_arm_typesupport_check")
    node.create_client(
        NavigateToPose.Impl.SendGoalService,
        "/nav_arm_typesupport_check/_action/send_goal",
    )
    node.destroy_node()
    rclpy.shutdown()
    print("ROS action typesupport check passed")


if __name__ == "__main__":
    main()
