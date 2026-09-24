"""Launch Gazebo Harmonic with indoor_bot spawned and its topics bridged to ROS 2."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Bring up headless Gazebo, spawn indoor_bot, and bridge its topics to ROS 2."""
    gazebo_pkg_share = get_package_share_directory('indoor_bot_gazebo')
    description_pkg_share = get_package_share_directory('indoor_bot_description')

    world_path = os.path.join(gazebo_pkg_share, 'worlds', 'indoor_room.sdf')
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'indoor_bot.urdf.xacro')
    bridge_config = os.path.join(gazebo_pkg_share, 'config', 'bridge.yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ),
        ),
        launch_arguments={
            'gz_args': f'-s -r --headless-rendering {world_path}',
        }.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_indoor_bot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'indoor_bot',
            '-z', '0.02',
        ],
        parameters=[{'use_sim_time': True}],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
    ])
