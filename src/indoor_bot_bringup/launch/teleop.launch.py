r"""
Launch teleop_twist_keyboard for manually driving indoor_bot.

teleop_twist_keyboard reads raw keystrokes from its controlling terminal.
`ros2 launch` multiplexes the output of every node it starts and does not
guarantee that stdin is connected to a real TTY, so keyboard input is not
reliable when this node runs under `ros2 launch` inside some terminal
emulators or CI/Docker setups. This launch file starts the node anyway,
with output='screen', for the common case where it works; it does not shell
out to a terminal emulator such as xterm to force a dedicated TTY, since
that would add a runtime dependency this project does not otherwise need.

If keyboard input does not register, run the node directly in your own
terminal instead::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/cmd_vel -p use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for teleop_twist_keyboard."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('cmd_vel', '/cmd_vel')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        teleop_node,
    ])
