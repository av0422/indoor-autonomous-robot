"""
Run the navigation trials: the full Nav2 stack plus the trial runner.

Includes nav.launch.py (Gazebo, Nav2, Foxglove) and starts trial_runner, which
writes one CSV row per trial. The whole launch shuts down when the runner exits.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, \
    RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Return the launch description for a batch of navigation trials."""
    bringup_pkg_share = get_package_share_directory('indoor_bot_bringup')

    declare_args = [
        DeclareLaunchArgument(
            'condition', default_value='unspecified',
            description='Label written to the condition column of the CSV'),
        DeclareLaunchArgument(
            'trials', default_value='4',
            description='Number of trials to run'),
        DeclareLaunchArgument(
            'seed', default_value='0',
            description='Seed for the start-yaw perturbation'),
        DeclareLaunchArgument(
            'output', default_value='results/trials.csv',
            description='CSV file the trial rows are appended to'),
    ]

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_share, 'launch', 'nav.launch.py'),
        ),
    )

    trial_runner_node = Node(
        package='indoor_bot_evaluation',
        executable='trial_runner',
        name='trial_runner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'condition': ParameterValue(LaunchConfiguration('condition'), value_type=str),
            'trials': ParameterValue(LaunchConfiguration('trials'), value_type=int),
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            'output': ParameterValue(LaunchConfiguration('output'), value_type=str),
        }],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=trial_runner_node,
            on_exit=[EmitEvent(event=Shutdown(reason='trial_runner finished'))],
        ),
    )

    return LaunchDescription(declare_args + [
        nav_launch,
        trial_runner_node,
        shutdown_when_done,
    ])
