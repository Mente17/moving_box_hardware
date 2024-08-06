from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moving_box_hardware',
            executable='test_sub.py',
            output='screen',
        ),
    ])