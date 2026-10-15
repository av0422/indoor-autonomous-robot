"""
Bring up Gazebo, Nav2 autonomous navigation, and Foxglove for indoor_bot.

Runs the normal (non-datagen) simulation, layers Nav2 localization and
navigation on top of the saved map, and starts foxglove_bridge so goals can
be sent and the costmaps and path inspected live from Foxglove Studio.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for an autonomous navigation session."""
    gazebo_pkg_share = get_package_share_directory('indoor_bot_gazebo')
    navigation_pkg_share = get_package_share_directory('indoor_bot_navigation')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'sim.launch.py'),
        ),
        launch_arguments={'datagen': 'false'}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg_share, 'launch', 'navigation.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        sim_launch,
        navigation_launch,
        foxglove_bridge_node,
    ])
