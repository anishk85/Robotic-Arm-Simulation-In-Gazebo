import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Launch argument for simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Find package share directories
    moveit_pkg_share = FindPackageShare("mycobot_moveit_config_new")
    description_pkg_share = FindPackageShare("mycobot_description")

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config_new")
        .to_moveit_configs()
    )

    # Gazebo world file
    gazebo_world_path = PathJoinSubstitution([moveit_pkg_share, "worlds", "empty.world"])

    # Launch Gazebo (server + client) using the standard launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": gazebo_world_path,
            "verbose": "true"
        }.items(),
    )

    # Robot description (xacro to urdf)
    robot_description_content = PathJoinSubstitution([moveit_pkg_share, "config", "mycobot_280.urdf.xacro"])
    robot_description = {
        "robot_description": robot_description_content
    }

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "mycobot_280",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"
        ],
        output="screen",
    )

    # MoveIt MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([moveit_pkg_share, "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Controller spawners (sequential with delays)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Event handlers for sequential controller startup
    spawn_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])],
        )
    )
    spawn_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(period=1.0, actions=[robot_controller_spawner])],
        )
    )
    spawn_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[TimerAction(period=1.0, actions=[gripper_controller_spawner])],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        rsp_node,
        spawn_entity,
        move_group_node,
        rviz_node,
        spawn_joint_state_broadcaster,
        spawn_robot_controller,
        spawn_gripper_controller,
    ])