#!/usr/bin/env python3
"""Minimal automatic demonstration of the navigation/arm interlock."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener


NS = "/a200_0000"
JOINTS = tuple(f"arm_0_joint_{index}" for index in range(1, 7))
STOW = (0.0, math.pi / 4, 5 * math.pi / 6, math.pi / 2, math.pi / 4, -math.pi / 2)
CARRY = (0.0, 0.0, 1.0, math.pi / 2, 1.8, -math.pi / 2)
READY = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class Demo(Node):
    def __init__(self):
        super().__init__("nav_arm_case_study_runner")
        self.moveit = ActionClient(self, MoveGroup, f"{NS}/move_action")
        self.nav2 = ActionClient(self, NavigateToPose, f"{NS}/navigate_to_pose")
        self.controller = ActionClient(
            self,
            FollowJointTrajectory,
            f"{NS}/arm_0_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.amcl = self.create_client(GetState, f"{NS}/amcl/get_state")
        self.navigator = self.create_client(GetState, f"{NS}/bt_navigator/get_state")
        self.initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, f"{NS}/initialpose", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait(self, future, timeout: float = 120.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception():
            raise RuntimeError("ROS operation timed out or failed")
        return future.result()

    def wait_active(self, client, label: str) -> None:
        if not client.wait_for_service(timeout_sec=120.0):
            raise RuntimeError(f"{label} is unavailable")
        while True:
            state = self.wait(client.call_async(GetState.Request()), 5.0)
            if state.current_state.id == State.PRIMARY_STATE_ACTIVE:
                return
            time.sleep(0.2)

    def prepare(self) -> None:
        for client, label in (
            (self.moveit, "MoveIt"),
            (self.nav2, "Nav2"),
            (self.controller, "arm controller"),
        ):
            if not client.wait_for_server(timeout_sec=120.0):
                raise RuntimeError(f"{label} is unavailable")
        self.wait_active(self.amcl, "AMCL")

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        for _ in range(3):
            self.initial_pose.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.2)
        deadline = time.monotonic() + 30.0
        while not self.tf_buffer.can_transform("map", "base_link", Time()):
            if time.monotonic() > deadline:
                raise RuntimeError("localization is not ready")
            rclpy.spin_once(self, timeout_sec=0.2)
        self.wait_active(self.navigator, "Nav2 navigator")

    @staticmethod
    def arm_goal(positions: tuple[float, ...]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm_0"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        constraints = Constraints()
        for name, position in zip(JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        return goal

    def move_arm(self, positions: tuple[float, ...], label: str) -> bool:
        self.get_logger().info(f"ARM -> {label}")
        handle = self.wait(self.moveit.send_goal_async(self.arm_goal(positions)))
        if not handle.accepted:
            return False
        result = self.wait(handle.get_result_async())
        return (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result.error_code.val == MoveItErrorCodes.SUCCESS
        )

    def navigate(self, target_x: float = 5.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target_x
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info(f"NAV -> x={target_x:.1f}")
        return self.wait(self.nav2.send_goal_async(goal))

    def run(self, monitored: bool) -> dict:
        if not self.move_arm(STOW, "STOW"):
            raise RuntimeError("cannot reach STOW")
        time.sleep(0.5)

        navigation = self.navigate()
        if not navigation.accepted:
            raise RuntimeError("navigation rejected after STOW")
        time.sleep(1.5)

        p2_blocked = not self.move_arm(READY, "READY during navigation")
        if p2_blocked != monitored:
            raise RuntimeError("unexpected P2 result")

        nav_result = self.wait(navigation.get_result_async(), 180.0)
        if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("navigation failed")
        time.sleep(0.5)

        if not self.move_arm(READY, "READY after navigation"):
            raise RuntimeError("arm remained blocked after navigation")
        return {"p2_blocked": p2_blocked, "success": True}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--monitoring", choices=("enabled", "disabled"), required=True)
    parser.add_argument("--result-file", type=Path, required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    demo = Demo()
    try:
        demo.prepare()
        result = demo.run(args.monitoring == "enabled")
    except Exception as error:  # report one clear failure to the shell
        result = {"success": False, "error": str(error)}
    finally:
        demo.destroy_node()
        rclpy.shutdown()

    args.result_file.write_text(json.dumps(result, indent=2) + "\n")
    print("CASE_RESULT " + json.dumps(result), flush=True)
    raise SystemExit(0 if result["success"] else 1)


if __name__ == "__main__":
    main()
