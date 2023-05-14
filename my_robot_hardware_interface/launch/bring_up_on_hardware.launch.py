import os
import xacro

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler

from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # robot_name = "rrbot"
    # robot_model_file = "rrbot.xacro"
    # package_name = "rrbot_unit3"



    # robot_description = os.path.join(get_package_share_directory(
    #     package_name), "urdf", robot_model_file)

    # robot_description_config = xacro.process_file(robot_description)


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("gz_rrbot_ros2"), "urdf", "rrbot_custom_hw.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config = os.path.join(
        get_package_share_directory(
            "my_robot_hardware_interface"), "config", "controller_configuration.yaml"
    )

    rviz_config = os.path.join(
        get_package_share_directory("gz_rrbot_ros2"), 
        "rviz", "custom_hw.rviz"
    )

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["forward_position_controller"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["joint_state_broadcaster",
        #                "--controller-manager", "/controller_manager"],
        # ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["forward_position_controller",
        #                "-c", "/controller_manager"],
        # ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["joint_trajectory_controller",
        #                "-c", "/controller_manager"],
        # ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output={
                "stdout": "screen",
                "stderr": "log",
            },
        )

    ])