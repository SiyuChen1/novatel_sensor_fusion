from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='novatel_sensor_fusion',
            namespace='/sync',
            executable='gnss_messages_sync',
            name='gnss_message_sync',
            parameters=[{
                'best_topic_name': '/best',
                'bestgnss_topic_name': '/bestgnss',
                'difference_best_bestgnss': '/diff_best_bestgnss',
                'difference_best_fused': '/diff_best_fused',
                'epsilon_sec': 0,
                'epsilon_nanosec': 1000
            }]
        )
    ])
