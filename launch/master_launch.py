import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    current_dir = get_package_share_directory("simple_navigation")
    lidar_auto_docking_dir = get_package_share_directory("lidar_auto_docking")
    goal_plotter_dir = get_package_share_directory("goal_plotter")

    ld = LaunchDescription()
    # launch navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "nav2_launch.py")
        )
    )
    # launch docking
    launch_lidar_auto_docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_auto_docking_dir, "launch", "nav2_dock.launch.py")
        )
    )
    # launch goal plotter
    launch_goal_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(goal_plotter_dir, "launch", "goal_manager.launch.py")
        )
    )
    # add actions here
    ld.add_action(launch_navigation)
    ld.add_action(launch_lidar_auto_docking)
    ld.add_action(launch_goal_manager)

    return ld
