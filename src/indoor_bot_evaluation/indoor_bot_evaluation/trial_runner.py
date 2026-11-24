"""Run repeated NavigateToPose trials against the simulator and log one CSV row each."""

import csv
import math
import os
import random
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

WORLD_NAME = 'indoor_room'
ROBOT_NAME = 'indoor_bot'
SPAWN_Z_M = 0.02
SETTLE_S = 3.0
CONTACT_DEBOUNCE_S = 0.5
START_YAW_JITTER_RAD = math.radians(10.0)
FLOOR_COLLISION_MARKER = 'ground_plane'

CSV_COLUMNS = [
    'trial', 'seed', 'condition', 'success', 'time_to_goal', 'path_length_m',
    'collision_count', 'recoveries', 'final_distance_error', 'real_time_factor',
    'status', 'pair',
]

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def yaw_to_quaternion(yaw):
    """Return (z, w) of the quaternion for a rotation about z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TrialRunner(Node):
    """Reset the robot, send a goal, and record the outcome, once per trial."""

    def __init__(self):
        """Declare parameters, load the goal pairs and create the ROS interfaces."""
        super().__init__(
            'trial_runner',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.declare_parameter('trials', 4)
        self.declare_parameter('seed', 0)
        self.declare_parameter('condition', 'unspecified')
        self.declare_parameter('output', 'results/trials.csv')
        self.declare_parameter('goals_file', '')
        self.declare_parameter('timeout_s', 90.0)
        self.declare_parameter('pair_index', -1)

        self.trials = int(self.get_parameter('trials').value)
        self.seed = int(self.get_parameter('seed').value)
        self.condition = self.get_parameter('condition').value
        self.output = self.get_parameter('output').value
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.pair_index = int(self.get_parameter('pair_index').value)
        goals_file = self.get_parameter('goals_file').value or os.path.join(
            get_package_share_directory('indoor_bot_evaluation'), 'config', 'goals.yaml')
        with open(goals_file) as handle:
            self.pairs = yaml.safe_load(handle)['pairs']
        if not self.pairs:
            raise ValueError(f'No start/goal pairs in {goals_file}')
        if self.pair_index >= len(self.pairs):
            raise ValueError(f'pair_index {self.pair_index} out of range')

        self.trial_active = False
        self.collision_count = 0
        self.last_contact_s = -math.inf
        self.path_length_m = 0.0
        self.recoveries = 0
        self.amcl_xy = None
        self.amcl_updates = 0
        self.contact_msgs = 0
        self.contact_names = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, amcl_qos)
        self.create_subscription(Contacts, '/contact', self.on_contact, 50)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in (
                '/global_costmap/clear_entirely_global_costmap',
                '/local_costmap/clear_entirely_local_costmap',
            )
        ]

    def now_s(self):
        """Return the current (simulated) time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, seconds):
        """Block for this many simulated seconds; give up after a generous wall limit."""
        start = self.now_s()
        wall_start = time.monotonic()
        while self.now_s() - start < seconds:
            if time.monotonic() - wall_start > seconds * 20.0 + 10.0:
                self.get_logger().warning('Simulated time is not advancing')
                return False
            time.sleep(0.02)
        return True

    def on_amcl_pose(self, msg):
        """Track the AMCL pose and accumulate path length while a trial runs."""
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.trial_active and self.amcl_xy is not None:
            self.path_length_m += math.dist(xy, self.amcl_xy)
        self.amcl_xy = xy
        self.amcl_updates += 1

    def on_contact(self, msg):
        """Count distinct contact episodes between the base and anything but the floor."""
        if not self.trial_active:
            return
        self.contact_msgs += 1
        for contact in msg.contacts:
            self.contact_names.add(f'{contact.collision1.name} | {contact.collision2.name}')
        if not any(self.is_obstacle_contact(contact) for contact in msg.contacts):
            return
        now = self.now_s()
        if now - self.last_contact_s > CONTACT_DEBOUNCE_S:
            self.collision_count += 1
        self.last_contact_s = now

    @staticmethod
    def is_obstacle_contact(contact):
        """Return True if neither collision of the contact is the ground plane."""
        return (FLOOR_COLLISION_MARKER not in contact.collision1.name
                and FLOOR_COLLISION_MARKER not in contact.collision2.name)

    def on_feedback(self, feedback_msg):
        """Remember the largest recovery count reported by Nav2."""
        self.recoveries = max(
            self.recoveries, int(feedback_msg.feedback.number_of_recoveries))

    def current_xy(self):
        """Return the map-frame position of base_footprint, or the last AMCL pose."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return self.amcl_xy
        return (transform.transform.translation.x, transform.transform.translation.y)

    def make_pose(self, x, y, yaw):
        """Return a map-frame PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
        return pose

    def stop_robot(self):
        """Publish zero velocity so the robot is at rest before it is moved."""
        for _ in range(3):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

    def teleport(self, x, y, yaw):
        """Move the robot with the Gazebo set_pose service; return True on success."""
        qz, qw = yaw_to_quaternion(yaw)
        request = (
            f'name: "{ROBOT_NAME}", position: {{x:{x:.3f}, y:{y:.3f}, z:{SPAWN_Z_M}}}, '
            f'orientation: {{z:{qz:.6f}, w:{qw:.6f}}}'
        )
        result = subprocess.run(
            [
                'gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', request,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        return 'true' in result.stdout

    def publish_initial_pose(self, x, y, yaw):
        """Publish the AMCL initial pose a few times so it is not missed."""
        for _ in range(3):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = yaw_to_quaternion(yaw)
            msg.pose.covariance[0] = 0.04
            msg.pose.covariance[7] = 0.04
            msg.pose.covariance[35] = 0.02
            self.initial_pose_pub.publish(msg)
            time.sleep(0.3)

    def clear_costmaps(self):
        """Clear both costmaps (best effort) so a previous trial leaves nothing behind."""
        for client in self.clear_clients:
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(ClearEntireCostmap.Request())
                deadline = time.monotonic() + 3.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)

    def reset_robot(self, start):
        """Teleport to the start pose, set the AMCL pose and wait for it to settle."""
        self.stop_robot()
        if not self.teleport(*start):
            self.get_logger().warning('Gazebo set_pose did not confirm the teleport')
        self.sleep_sim(0.5)
        updates_before = self.amcl_updates
        self.publish_initial_pose(*start)
        self.sleep_sim(SETTLE_S)
        self.clear_costmaps()
        if self.amcl_updates == updates_before:
            self.get_logger().warning('No AMCL pose update after the initial pose')

    def wait_for_stack(self):
        """Wait until the Nav2 action server, AMCL and the sim clock are available."""
        while not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for the navigate_to_pose action server')
        while self.initial_pose_pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for AMCL to subscribe to /initialpose')
            time.sleep(1.0)
        while self.now_s() <= 0.0:
            time.sleep(0.1)

    def run_trial(self, index):
        """Run one trial and return its CSV row as a dict."""
        pair_number = self.pair_index if self.pair_index >= 0 else index % len(self.pairs)
        pair = self.pairs[pair_number]
        rng = random.Random(self.seed * 1000003 + index)
        jitter = rng.uniform(-START_YAW_JITTER_RAD, START_YAW_JITTER_RAD)
        start = (pair['start']['x'], pair['start']['y'], pair['start']['yaw'] + jitter)
        goal = pair['goal']
        self.get_logger().info(
            f'Trial {index}: pair {pair_number} ({pair["name"]}), '
            f'start yaw jitter {math.degrees(jitter):+.1f} deg')

        self.reset_robot(start)

        self.collision_count = 0
        self.last_contact_s = -math.inf
        self.contact_msgs = 0
        self.contact_names = set()
        self.path_length_m = 0.0
        self.recoveries = 0
        if self.amcl_xy is None:
            self.amcl_xy = (start[0], start[1])

        row = {
            'trial': index, 'seed': self.seed, 'condition': self.condition,
            'success': 0, 'time_to_goal': 0.0, 'path_length_m': 0.0,
            'collision_count': 0, 'recoveries': 0,
            'final_distance_error': '', 'real_time_factor': '', 'status': 'REJECTED',
            'pair': pair_number,
        }

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(goal['x'], goal['y'], goal['yaw'])
        self.trial_active = True
        sim_start = self.now_s()
        wall_start = time.monotonic()
        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.on_feedback)
        while not send_future.done():
            time.sleep(0.02)
        goal_handle = send_future.result()

        status = 'REJECTED'
        sim_end = sim_start
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            wall_limit = self.timeout_s * 20.0 + 30.0
            timed_out = False
            while not result_future.done():
                if self.now_s() - sim_start > self.timeout_s:
                    timed_out = True
                    break
                if time.monotonic() - wall_start > wall_limit:
                    timed_out = True
                    break
                time.sleep(0.05)
            sim_end = self.now_s()
            if timed_out:
                status = 'TIMEOUT'
                cancel_future = goal_handle.cancel_goal_async()
                deadline = time.monotonic() + 10.0
                while not cancel_future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
            else:
                code = result_future.result().status
                status = STATUS_NAMES.get(code, f'STATUS_{code}')
        wall_end = time.monotonic()
        self.trial_active = False
        self.stop_robot()

        final_xy = self.current_xy()
        if final_xy is not None:
            row['final_distance_error'] = round(
                math.dist(final_xy, (goal['x'], goal['y'])), 3)
        sim_elapsed = sim_end - sim_start
        wall_elapsed = wall_end - wall_start
        row.update({
            'success': int(status == 'SUCCEEDED'),
            'time_to_goal': round(sim_elapsed, 3),
            'path_length_m': round(self.path_length_m, 3),
            'collision_count': self.collision_count,
            'recoveries': self.recoveries,
            'real_time_factor': round(sim_elapsed / wall_elapsed, 3) if wall_elapsed > 0 else '',
            'status': status,
        })
        return row

    def run(self):
        """Run all trials, appending one flushed CSV row after each."""
        output_dir = os.path.dirname(os.path.abspath(self.output))
        os.makedirs(output_dir, exist_ok=True)
        self.wait_for_stack()
        with open(self.output, 'a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
            if handle.tell() == 0:
                writer.writeheader()
                handle.flush()
            for index in range(self.trials):
                row = self.run_trial(index)
                writer.writerow(row)
                handle.flush()
                os.fsync(handle.fileno())
                self.get_logger().info(f'Trial {index} done: {row}')
                self.get_logger().info(
                    f'Trial {index} raw /contact messages: {self.contact_msgs}, '
                    f'collision pairs seen: {sorted(self.contact_names)}')


def main(args=None):
    """Spin the node in a thread and run the trials on the main thread."""
    rclpy.init(args=args)
    node = TrialRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
