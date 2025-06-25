import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def load_params(profile_name):
    config_file = os.path.join(
        get_package_share_directory('battery_simulator'),
        'config',
        'battery_params.yaml'
    )
    with open(config_file, 'r') as f:
        all_params = yaml.safe_load(f)
    return all_params.get(profile_name, {}).get('ros__parameters', {})

def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        'battery_profile',
        default_value='default',
        description='Battery profile name from battery_params.yaml'
    )

    battery_profile = LaunchConfiguration('battery_profile')

    def battery_node_fn(context):
        profile_name = battery_profile.perform(context)
        params = load_params(profile_name)

        return [Node(
            package='battery_simulator',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[params]
        )]

    

    return LaunchDescription([
        profile_arg,
        OpaqueFunction(function=battery_node_fn)
    ])
