import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'moving_box_hardware'
    ld = LaunchDescription()

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Micro ROS Agent', '-e', 'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0', 'baudrate=115200'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'RPLIDAR', '-e', 'ros2', 'launch', 'rplidar_ros', 'rplidar_c1_launch.py', 'use_sim_time:=false'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Moving Box Driver', '-e', 'ros2', 'launch', package_name, 'moving_box.launch.py'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Joystick', '-e', 'ros2', 'launch', package_name, 'joystick.launch.py'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Twist Stamper', '-e', 'ros2', 'run', 'twist_stamper', 'twist_stamper', '-ros-args', '-r', 'cmd_vel_in:=cmd_vel', '-r', 'cmd_vel_out:=moving_box_controller/cmd_vel', '-p', 'frame_id:=base_link'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Online Async SLAM', '-e', 'ros2', 'launch', package_name, 'online_async_launch.py', 'use_sim_time:=false'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['xterm', '-T', 'Nav2', '-e', 'ros2', 'launch', package_name, 'navigation_launch.py', 'use_sim_time:=false'],
        output='screen'
    ))

    return ld
