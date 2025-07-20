#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    robot_desc_pkg = get_package_share_directory('two_wheels_bot_description')
    robot_gazebo_pkg = get_package_share_directory('two_wheels_bot_gazebo')

    # Paths
    empty_world = os.path.join(robot_gazebo_pkg, 'worlds', 'empty_world.world')
    controller_config = os.path.join(robot_desc_pkg, 'config', 'my_controllers.yaml')
    twist_mux_config = os.path.join(robot_gazebo_pkg, 'config', 'twist_mux.yaml')

    # Launch args for robot spawn position
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0', description='Robot initial X')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0', description='Robot initial Y')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.01', description='Robot initial Z')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Gazebo server and client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': empty_world}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
    )

    # Spawn robot entity from robot_description's published topic (URDF must be published externally)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_car',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # Controllers
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        parameters=[controller_config],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        parameters=[controller_config],
        output='screen'
    )

    # Twist mux node to handle velocity commands
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_sim_time_arg,
        gzserver,
        gzclient,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        twist_mux_node
    ])
