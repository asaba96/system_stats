import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    seperate_stats_arg = DeclareLaunchArgument("seperate_stats", default_value="True")

    sys_stats = Node(
        package="system_stats",
        executable="system_monitor.py",
        parameters=[
            {
                "seperate_stats": LaunchConfiguration("seperate_stats"),
            }
        ],
    )

    return LaunchDescription(
        [
            seperate_stats_arg,
            sys_stats,
        ]
    )
