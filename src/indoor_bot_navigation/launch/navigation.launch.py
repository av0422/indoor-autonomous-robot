"""
Launch Nav2 localization and navigation for indoor_bot against a saved map.

Includes nav2_bringup's localization_launch.py (map_server + amcl). The
navigation half does NOT include nav2_bringup's own navigation_launch.py:
on this Jazzy install that file's lifecycle_nodes list hardcodes
route_server, collision_monitor and docking_server, none of which has a
section in nav2_params.yaml, so lifecycle_manager_navigation aborts bringup
("parameter 'observation_sources' is not initialized") before activating
anything. There is no launch argument on the installed file to opt those
nodes out, so this file starts only the seven servers this project actually
configures, plus their own lifecycle manager, mirroring the Node
definitions navigation_launch.py uses for those seven exactly.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
CMD_VEL_REMAPPINGS = TF_REMAPPINGS + [('cmd_vel', 'cmd_vel_nav')]
# velocity_smoother takes controller_server's output on its default input
# ('cmd_vel', remapped above to 'cmd_vel_nav') but, unlike every other
# node here, does not publish its smoothed result back on 'cmd_vel' by
# default — it publishes 'cmd_vel_smoothed', which nothing else subscribes
# to. The ros_gz_bridge only bridges plain '/cmd_vel' (Twist, unstamped;
# docs/INTERFACES.md gotcha a), so that output needs its own remap too.
VELOCITY_SMOOTHER_REMAPPINGS = CMD_VEL_REMAPPINGS + [('cmd_vel_smoothed', 'cmd_vel')]

NAVIGATION_LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    """Return the launch description for Nav2 localization and navigation."""
    navigation_pkg_share = get_package_share_directory('indoor_bot_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(navigation_pkg_share, 'maps', 'indoor_room.yaml')
    default_params_file = os.path.join(navigation_pkg_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the saved map YAML to localize against',
    )
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file',
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the lifecycle nodes',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py'),
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    node_parameters = [params_file, {'use_sim_time': use_sim_time}]

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=node_parameters,
        remappings=CMD_VEL_REMAPPINGS,
    )
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=node_parameters,
        remappings=TF_REMAPPINGS,
    )
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=node_parameters,
        remappings=TF_REMAPPINGS,
    )
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=node_parameters,
        remappings=CMD_VEL_REMAPPINGS,
    )
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=node_parameters,
        remappings=TF_REMAPPINGS,
    )
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=node_parameters,
        remappings=TF_REMAPPINGS,
    )
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=node_parameters,
        remappings=VELOCITY_SMOOTHER_REMAPPINGS,
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': NAVIGATION_LIFECYCLE_NODES,
        }],
    )

    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        localization_launch,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ])
