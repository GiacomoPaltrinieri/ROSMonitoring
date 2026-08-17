#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


CASE_ROOT = os.environ.get("CASE_STUDY_ROOT", str(Path.cwd()))


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

    simulation = include(
        "clearpath_gz",
        "simulation.launch.py",
        {"setup_path": setup_path, "world": world, "use_sim_time": "true"},
    )
    moveit = include(
        "clearpath_manipulators",
        "moveit.launch.py",
        {"setup_path": setup_path, "use_sim_time": "true"},
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

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("setup_path", default_value=f"{CASE_ROOT}/clearpath"),
            DeclareLaunchArgument("world", default_value="warehouse"),
            DeclareLaunchArgument("startup_delay", default_value="10.0"),
            simulation,
            TimerAction(period=startup_delay, actions=[moveit, nav2, localization]),
        ]
    )
