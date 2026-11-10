#!/usr/bin/env python3
"""
Drive indoor_bot around the room so slam_toolbox can build a map.

Follows the room's perimeter by driving forward until an obstacle closes to
within a block-distance threshold in the +-30 degree forward window, then
rotating left in place until that window clears again. For the first four
minutes of sim time the block distance is 0.6 m, the same threshold used as
the emergency-stop safety check; after that it widens to 1.5 m, so later
turns are triggered earlier and the perimeter loop spirals inward toward
the middle of the room. Publishes geometry_msgs/Twist on /cmd_vel at 10 Hz
and runs for 10 minutes of sim time before publishing a zero Twist and
exiting.

Run inside the indoor-bot container, from /ws, with the ROS 2 and venv
environments sourced, while the simulation is already running::

    python3 src/indoor_bot_evaluation/scripts/mapping_drive.py
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LINEAR_SPEED_MPS = 0.15
ANGULAR_SPEED_RAD_S = 0.3
CONTROL_PERIOD_S = 0.1
FORWARD_HALF_WINDOW_RAD = math.radians(30.0)
BLOCK_RANGE_EARLY_M = 0.6
BLOCK_RANGE_SPIRAL_M = 1.5
CLEAR_RANGE_M = 1.2
SPIRAL_START_S = 4.0 * 60.0
RUN_DURATION_S = 10.0 * 60.0


class MappingDriveNode(Node):
    """Drive indoor_bot in a perimeter-then-spiral pattern for mapping."""

    def __init__(self):
        """Set up the /cmd_vel publisher and the /scan subscription."""
        super().__init__('mapping_drive')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.min_forward_range = math.inf
        self.recovering = False

    def _on_scan(self, msg):
        """Track the minimum finite range in the +-30 degree forward window."""
        ranges = [
            r for i, r in enumerate(msg.ranges)
            if math.isfinite(r)
            and abs(msg.angle_min + i * msg.angle_increment) <= FORWARD_HALF_WINDOW_RAD
        ]
        self.min_forward_range = min(ranges) if ranges else math.inf

    def drive(self, block_range_m):
        """Publish the next velocity command for the given block distance."""
        if self.recovering:
            if self.min_forward_range > CLEAR_RANGE_M:
                self.recovering = False
        elif self.min_forward_range < block_range_m:
            self.recovering = True

        cmd = Twist()
        if self.recovering:
            cmd.angular.z = ANGULAR_SPEED_RAD_S
        else:
            cmd.linear.x = LINEAR_SPEED_MPS
        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """Publish a single zero Twist to bring the robot to a halt."""
        self.cmd_vel_pub.publish(Twist())


def main():
    """Run the perimeter-then-spiral route for RUN_DURATION_S of sim time."""
    rclpy.init()
    node = MappingDriveNode()
    start_time = node.get_clock().now()

    while True:
        elapsed_s = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed_s >= RUN_DURATION_S:
            break
        block_range_m = (
            BLOCK_RANGE_SPIRAL_M if elapsed_s >= SPIRAL_START_S else BLOCK_RANGE_EARLY_M
        )
        node.drive(block_range_m)
        rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)

    node.stop()
    node.get_logger().info(f'Mapping drive finished after {elapsed_s:.0f} s of sim time.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
