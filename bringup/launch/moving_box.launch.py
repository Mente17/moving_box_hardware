import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name='moving_box_hardware'

    # Declare arguments
    declared_arguments = []

    # Initialize Arguments
    #gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("moving_box_hardware"), "urdf", "moving_box.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("moving_box_hardware"),
            "config",
            "moving_box_controllers.yaml",
        ]
    )
#    rviz_config_file = PathJoinSubstitution(
#        [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
#    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/moving_box_controller/cmd_vel_unstamped')]
        )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            #  ("/moving_box_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/moving_box_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
#        arguments=["-d", rviz_config_file],
#        condition=IfCondition(gui),
#    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["moving_box_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
#    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_state_broadcaster_spawner,
#            on_exit=[rviz_node],
#        )
#    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        twist_mux,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
#        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

