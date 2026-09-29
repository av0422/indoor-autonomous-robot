"""
Bring up Gazebo, async SLAM Toolbox mapping, and Foxglove for indoor_bot.

Runs the normal (non-datagen) simulation, layers async_slam_toolbox_node on
top to build an occupancy grid from /scan, and starts foxglove_bridge so the
map and TF tree can be inspected live from Foxglove Studio.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for a mapping session."""
    gazebo_pkg_share = get_package_share_directory('indoor_bot_gazebo')
    navigation_pkg_share = get_package_share_directory('indoor_bot_navigation')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'sim.launch.py'),
        ),
        launch_arguments={'datagen': 'false'}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg_share, 'launch', 'slam.launch.py'),
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        slam_launch,
        foxglove_bridge_node,
    ])
