#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap


CASE_ROOT = os.environ.get("CASE_STUDY_ROOT", str(Path.cwd()))
ROSMONITORING_ROOT = os.environ.get(
    "ROSMONITORING_ROOT", str(Path(CASE_ROOT).resolve().parents[1])
)
ARM_SEND_GOAL = (
    "/a200_0000/arm_0_joint_trajectory_controller/"
    "follow_joint_trajectory/_action/send_goal"
)
GRIPPER_SEND_GOAL = (
    "/a200_0000/arm_0_gripper_controller/gripper_cmd/_action/send_goal"
)


def include(package: str, launch_file: str, arguments: dict[str, object]):
    source = PythonLaunchDescriptionSource(
        f"{get_package_share_directory(package)}/launch/{launch_file}"
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=[(name, value) for name, value in arguments.items()],
    )


def generate_launch_description() -> LaunchDescription:
    setup_path = LaunchConfiguration("setup_path")
    world = LaunchConfiguration("world")
    startup_delay = LaunchConfiguration("startup_delay")
    case_root = LaunchConfiguration("case_root")
    rosmonitoring_root = LaunchConfiguration("rosmonitoring_root")

    simulation = include(
        "clearpath_gz",
        "simulation.launch.py",
        {"setup_path": setup_path, "world": world, "use_sim_time": "true"},
    )
    moveit = GroupAction(
        [
            SetRemap(src=ARM_SEND_GOAL, dst=ARM_SEND_GOAL + "_mon"),
            SetRemap(src=GRIPPER_SEND_GOAL, dst=GRIPPER_SEND_GOAL + "_mon"),
            include(
                "clearpath_manipulators",
                "moveit.launch.py",
                {"setup_path": setup_path, "use_sim_time": "true"},
            ),
        ]
    )
    nav2 = include(
        "clearpath_nav2_demos",
        "nav2.launch.py",
        {"setup_path": setup_path, "use_sim_time": "true"},
    )
    localization = include(
        "clearpath_nav2_demos",
        "localization.launch.py",
        {"setup_path": setup_path, "use_sim_time": "true"},
    )

    oracle = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([case_root, ".venv", "bin", "python"]),
            PathJoinSubstitution(
                [rosmonitoring_root, "oracle", "TLOracle", "oracle.py"]
            ),
            "--property",
            "nav_arm_property",
            "--online",
            "--discrete",
        ],
        output="screen",
        additional_env={
            "PYTHONUNBUFFERED": "1",
            "PYTHONPATH": PathJoinSubstitution([case_root, "oracle"]),
        },
    )
    monitor = Node(
        package="monitor",
        executable="nav_arm_interlock_monitor",
        name="nav_arm_interlock_monitor",
        output="screen",
    )
    joint_state_sampler = Node(
        package="nav_arm_case_study",
        executable="joint_state_sampler",
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("case_root", default_value=CASE_ROOT),
            DeclareLaunchArgument(
                "rosmonitoring_root", default_value=ROSMONITORING_ROOT
            ),
            DeclareLaunchArgument("setup_path", default_value=f"{CASE_ROOT}/clearpath"),
            DeclareLaunchArgument("world", default_value="warehouse"),
            DeclareLaunchArgument("startup_delay", default_value="10.0"),
            oracle,
            joint_state_sampler,
            TimerAction(period=1.0, actions=[monitor]),
            simulation,
            TimerAction(period=startup_delay, actions=[moveit, nav2, localization]),
        ]
    )
