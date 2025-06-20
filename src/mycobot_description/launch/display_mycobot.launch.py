from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    declared_arguments = []

    # Argument: model path
    declared_arguments.append(
        DeclareLaunchArgument(
            name='model',
            default_value=PathJoinSubstitution([
                FindPackageShare('mycobot_description'),
                'urdf',
                'mycobot_280_wrapper.xacro'
            ]),
            description='Absolute path to robot URDF (XACRO) file'
        )
    )

    model_path = LaunchConfiguration('model')

    return LaunchDescription(declared_arguments + [

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', model_path])
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('mycobot_description'),
                'rviz',
                'display.rviz'  # optional: only if you have a saved config
            ])]
        )
    ])

