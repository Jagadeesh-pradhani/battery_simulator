import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap, Node


def generate_launch_description():

    battery_node = Node(
            package="battery_simulator",
            executable="battery_node",
            name="battery_node",
            output="screen",
            parameters=[{"discharge_model": "linear",
                         "max_voltage": 12.6,
                         "min_voltage": 11.4,
                         "base_voltage": 12.0,
                         "initial_percent": 100,
                         "discharge_current": 10000,
                         "recharge_current": 2400,
                         "base_power_consumption": 25000,
                         "motors_power_consumption": 30000,
                         "num_batteries": 2,
                         "cmd_vel_topic": "/simple_drone/cmd_vel",
                         "verbose": True
                         }] 
        )

    return LaunchDescription([
        battery_node
    ])