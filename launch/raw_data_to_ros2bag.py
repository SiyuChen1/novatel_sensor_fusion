import launch
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetLaunchConfiguration('config_file', value=[launch.substitutions.ThisLaunchFileDir(), '/../config/raw_data.yaml']),
        Node(
            package='novatel_sensor_fusion',
            namespace='/ros2bag',
            executable='read_raw_data_to_bag_group_by_timestamp.py',
            name='raw_data_to_ros2bag',
            parameters=[launch.substitutions.LaunchConfiguration('config_file')]
        )
    ])
