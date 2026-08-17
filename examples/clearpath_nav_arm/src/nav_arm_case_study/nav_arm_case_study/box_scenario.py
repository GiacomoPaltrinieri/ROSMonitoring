#!/usr/bin/env python3
"""Minimal demonstration of a box drop prevented by P3."""

import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args

from nav_arm_case_study.scenario_runner import CARRY, Demo


NS = "/a200_0000"
GROUND_PICK = (0.0, -2.207, -0.624, 1.571, 1.558, -1.571)
CLOSE_POSITION = 0.30
DESTINATION_X = 5.0
OPEN_AFTER_DISTANCE = 2.0
MIN_MOVING_SPEED = 0.03
CARRY_HOLD_SECONDS = 2.0

BOX = """<sdf version="1.9">
<model name="cargo_box"><link name="box">
<inertial><mass>0.05</mass><inertia><ixx>0.00004</ixx><iyy>0.00004</iyy><izz>0.00004</izz></inertia></inertial>
<visual name="visual"><geometry><box><size>0.05 0.07 0.20</size></box></geometry>
<material><ambient>0.62 0.42 0.22 1</ambient><diffuse>0.76 0.55 0.30 1</diffuse></material></visual>
<visual name="label"><pose>0.0255 0 0 0 0 0</pose>
<geometry><box><size>0.001 0.035 0.08</size></box></geometry>
<material><ambient>0.85 0.85 0.80 1</ambient><diffuse>0.95 0.95 0.90 1</diffuse></material></visual>
<collision name="collision"><geometry><box><size>0.05 0.07 0.20</size></box></geometry></collision>
</link></model></sdf>"""


def command(*arguments: str) -> str:
    result = subprocess.run(arguments, text=True, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


class BoxDemo(Demo):
    def __init__(self):
        super().__init__()
        self.gripper = ActionClient(
            self, GripperCommand, f"{NS}/arm_0_gripper_controller/gripper_cmd"
        )
        self.position = None
        self.speed = 0.0
        self.create_subscription(
            Odometry, f"{NS}/platform/odom", self.store_position, 1
        )

    def store_position(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.position = (position.x, position.y)
        velocity = message.twist.twist.linear
        self.speed = math.hypot(velocity.x, velocity.y)

    def prepare(self) -> None:
        self.spawn_box()
        super().prepare()
        if not self.gripper.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("gripper is unavailable")

    def grip(self, position: float, wait_for_result: bool = True) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        handle = self.wait(self.gripper.send_goal_async(goal), 15.0)
        if not handle.accepted:
            return False
        if wait_for_result:
            self.wait(handle.get_result_async(), 30.0)
        return True

    def robot_position(self) -> tuple[float, float]:
        deadline = time.monotonic() + 5.0
        while self.position is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position is None:
            raise RuntimeError("robot odometry is unavailable")
        return self.position

    def wait_until_robot_moved(
        self, start: tuple[float, float]
    ) -> tuple[float, float, float]:
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            current = self.robot_position()
            distance = math.hypot(current[0] - start[0], current[1] - start[1])
            if distance >= OPEN_AFTER_DISTANCE and self.speed >= MIN_MOVING_SPEED:
                return distance, abs(current[1] - start[1]), self.speed
        current = self.robot_position()
        distance = math.hypot(current[0] - start[0], current[1] - start[1])
        raise RuntimeError(
            f"robot moved only {distance:.2f} m before the release"
        )

    @staticmethod
    def spawn_box() -> None:
        command(
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "warehouse", "-string", BOX, "-name", "cargo_box",
            "-x", "0.553", "-y", "0.049", "-z", "0.10",
        )

    @staticmethod
    def box_height() -> float:
        output = command("gz", "model", "-m", "cargo_box", "-p")
        match = re.search(r"Pose.*?\[\s*[-0-9.e]+\s+[-0-9.e]+\s+([-0-9.e]+)", output, re.S)
        if not match:
            raise RuntimeError("cannot read box position")
        return float(match.group(1))

    @staticmethod
    def attach_box() -> None:
        robot = command("gz", "model", "-m", "a200_0000/robot", "-p")
        robot_id = re.search(r"Model: \[(\d+)\]", robot)
        if not robot_id:
            raise RuntimeError("cannot find simulated robot")
        plugin = (
            "<parent_link>arm_0_gripper_left_finger_dist_link</parent_link>"
            "<child_model>cargo_box</child_model><child_link>box</child_link>"
            "<detach_topic>/cargo_box/detach</detach_topic>"
        )
        request = (
            f"entity: {{id: {robot_id.group(1)}, type: MODEL}}, "
            "plugins: {name: \"gz::sim::systems::DetachableJoint\", "
            "filename: \"gz-sim-detachable-joint-system\", "
            f"innerxml: \"{plugin}\"}}"
        )
        try:
            subprocess.run(
                (
                    "gz", "service", "-s", "/world/warehouse/entity/system/add",
                    "--reqtype", "gz.msgs.EntityPlugin_V",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "1000",
                    "--req", request,
                ),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
            )
        except subprocess.TimeoutExpired:
            pass

    @staticmethod
    def detach_box() -> None:
        command(
            "gz", "topic", "-t", "/cargo_box/detach",
            "-m", "gz.msgs.Empty", "-p", "unused: true", "-d", "1.0",
        )

    def run(self, monitored: bool) -> dict:
        print("1. The box is ready on the floor", flush=True)
        if not self.grip(0.0):
            raise RuntimeError("cannot open the gripper")

        print("2. Pick up the box and move to CARRY", flush=True)
        if not self.move_arm(GROUND_PICK, "GROUND PICK"):
            raise RuntimeError("cannot reach the box")
        if not self.grip(CLOSE_POSITION, wait_for_result=False):
            raise RuntimeError("cannot close the gripper")
        time.sleep(2.0)
        before = self.box_height()
        self.attach_box()
        carry_reached = self.move_arm(CARRY, "CARRY")
        if not carry_reached:
            raise RuntimeError("cannot reach CARRY")
        print("   CARRY reached; navigation is now permitted", flush=True)
        time.sleep(CARRY_HOLD_SECONDS)
        after = self.box_height()
        box_picked = after > before + 0.08
        if not box_picked:
            raise RuntimeError(f"box height did not increase: {before:.3f} -> {after:.3f}")
        before_drop = after
        print("3. Navigate forward while carrying the box", flush=True)
        navigation = self.navigate(DESTINATION_X)
        if not navigation.accepted:
            raise RuntimeError("navigation rejected in CARRY")
        start = self.robot_position()
        distance, lateral_deviation, moving_speed = self.wait_until_robot_moved(start)
        print(
            f"   Robot moved {distance:.2f} m with "
            f"{lateral_deviation:.2f} m lateral deviation; "
            f"speed is {moving_speed:.2f} m/s",
            flush=True,
        )

        print("4. Try to open the gripper while the robot is moving", flush=True)
        gripper_opened = self.grip(0.0, wait_for_result=False)
        p3_blocked = not gripper_opened
        if p3_blocked != monitored:
            raise RuntimeError("unexpected P3 result")
        if gripper_opened:
            self.detach_box()
        time.sleep(2.0)
        after_drop = self.box_height()
        box_dropped = after_drop < before_drop - 0.15
        if box_dropped != (not monitored):
            raise RuntimeError(
                f"unexpected box height: {before_drop:.3f} -> {after_drop:.3f}"
            )
        print(f"   P3 result: {'BLOCKED' if p3_blocked else 'BOX DROPPED'}", flush=True)

        print("5. Continue to the destination", flush=True)
        nav_result = self.wait(navigation.get_result_async(), 180.0)
        if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("navigation did not reach the destination")
        print("   Destination reached", flush=True)
        time.sleep(2.0)

        return {
            "box_picked": box_picked,
            "carry_reached": carry_reached,
            "p3_blocked": p3_blocked,
            "box_dropped": box_dropped,
            "forward_distance": round(distance, 3),
            "speed_when_open_sent": round(moving_speed, 3),
            "lateral_deviation": round(lateral_deviation, 3),
            "destination_reached": True,
            "success": True,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--monitoring", choices=("enabled", "disabled"), required=True)
    parser.add_argument("--result-file", type=Path, required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    demo = BoxDemo()
    try:
        demo.prepare()
        result = demo.run(args.monitoring == "enabled")
    except Exception as error:
        result = {"success": False, "error": str(error)}
    finally:
        demo.destroy_node()
        rclpy.shutdown()

    args.result_file.write_text(json.dumps(result, indent=2) + "\n")
    print("BOX_RESULT " + json.dumps(result), flush=True)
    raise SystemExit(0 if result["success"] else 1)


if __name__ == "__main__":
    main()
