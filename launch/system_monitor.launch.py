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

    # args that can be set from the command line or a default will be used
    pub_nodes_arg = DeclareLaunchArgument(
        "pub_nodes", default_value="False"
    )
    pub_dead_nodes_arg = DeclareLaunchArgument(
        "pub_dead_nodes", default_value="True"
    )
    seperate_stats_arg = DeclareLaunchArgument(
        "seperate_stats", default_value="True"
    )
 

  

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    sys_stats = Node(
            package='system_stats',
            executable='system_monitor.py',
            parameters=[{
                "pub_nodes": LaunchConfiguration('pub_nodes'),
                "pub_dead_nodes": LaunchConfiguration('pub_dead_nodes'),
                "seperate_stats": LaunchConfiguration('seperate_stats'),
            }]
        )

   

    return LaunchDescription([
        pub_nodes_arg,
        pub_dead_nodes_arg,
        seperate_stats_arg,
        sys_stats,
    ])