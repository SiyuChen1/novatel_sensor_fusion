from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='novatel_sensor_fusion',
            namespace='/ros2bag',
            executable='read_raw_data_to_bag_group_by_timestamp.py',
            name='raw_data_to_ros2bag',
            parameters=[{
                'file_path': '/home/siyuchen/Documents/20230907/20230907_122749/IMUData_20230907_122749.log',
                'ros2bag_name': '20230907_Laurensberg'
            }]
        )
    ])
