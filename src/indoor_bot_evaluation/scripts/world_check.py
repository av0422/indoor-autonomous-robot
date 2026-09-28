#!/usr/bin/env python3
"""
Measure LiDAR and camera+YOLO visibility of each world object.

For every object in indoor_bot_gazebo/worlds/indoor_room.sdf and every
standoff distance in (1.0, 2.0, 3.0) m, teleports the robot to a pose
facing the object, captures one camera frame and one LiDAR scan, and
reports what each sensor sees. Requires the simulation to already be
running (`ros2 launch indoor_bot_gazebo sim.launch.py`); this script does
not start or stop it.

Run inside the indoor-bot container, from /ws, with the ROS 2 and venv
environments sourced:

    python3 src/indoor_bot_evaluation/scripts/world_check.py
"""

import math
from pathlib import Path
import statistics
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from ultralytics import YOLO

# name -> (x, y), taken from indoor_room.sdf. tier is only used for the
# printed report; it does not change how a pose or measurement is made.
OBJECTS = {
    'low_backpack': ((1.2, 0.3), 'low'),
    'low_suitcase': ((-0.8, 0.8), 'low'),
    'low_sports_ball': ((1.0, -0.5), 'low'),
    'chair_1': ((1.8, 1.2), 'tall'),
    'chair_2': ((-1.5, 1.8), 'tall'),
    'chair_3': ((0.5, -2.3), 'tall'),
    'table_1': ((-2.0, -1.5), 'tall'),
    'potted_plant_1': ((2.3, -1.0), 'tall'),
}

DISTANCES_M = (1.0, 2.0, 3.0)
ROOM_HALF_EXTENT_M = 4.0
WALL_MARGIN_M = 0.6
POSE_LIMIT_M = ROOM_HALF_EXTENT_M - WALL_MARGIN_M
MIN_OBJECT_CLEARANCE_M = 0.4
SETTLE_TIME_S = 2.5
FORWARD_HALF_WINDOW_RAD = math.radians(8.0)
CAPTURE_TIMEOUT_S = 5.0

MODEL_PATH = 'yolo11n.pt'
YOLO_IMGSZ = 320
YOLO_CONF = 0.05
TARGET_CLASSES = [
    'chair', 'potted plant', 'dining table', 'couch', 'backpack',
    'suitcase', 'sports ball', 'bottle', 'person',
]

RESULTS_DIR = Path('results/world_check')


def clamp(value, low, high):
    """Clamp value into [low, high]."""
    return max(low, min(high, value))


def pose_for(object_name, object_xy, distance_m):
    """
    Return (x, y, yaw) for a robot standing distance_m from the object.

    The robot sits on the line from the room centre through the object,
    clamped to stay inside the walls, and is skipped (returns None) if
    that lands within MIN_OBJECT_CLEARANCE_M of a different object.
    """
    ox, oy = object_xy
    norm = math.hypot(ox, oy)
    ux, uy = (ox / norm, oy / norm) if norm > 1e-6 else (1.0, 0.0)

    rx = clamp(ox - ux * distance_m, -POSE_LIMIT_M, POSE_LIMIT_M)
    ry = clamp(oy - uy * distance_m, -POSE_LIMIT_M, POSE_LIMIT_M)

    for other_name, (other_xy, _tier) in OBJECTS.items():
        if other_name == object_name:
            continue
        if math.hypot(rx - other_xy[0], ry - other_xy[1]) < MIN_OBJECT_CLEARANCE_M:
            return None

    yaw = math.atan2(oy - ry, ox - rx)
    return rx, ry, yaw


def teleport(name, x, y, yaw):
    """Zero /cmd_vel is the caller's job; this only issues the gz set_pose."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f'name: "{name}", position: {{x:{x:.3f}, y:{y:.3f}, z:0.05}}, '
        f'orientation: {{z:{qz:.6f}, w:{qw:.6f}}}'
    )
    subprocess.run(
        [
            'gz', 'service', '-s', '/world/indoor_room/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def analyze_scan(scan):
    """Return (min_range, median_range) within +-8 deg of straight ahead."""
    forward = [
        r for i, r in enumerate(scan.ranges)
        if abs(scan.angle_min + i * scan.angle_increment) <= FORWARD_HALF_WINDOW_RAD
        and math.isfinite(r)
    ]
    if not forward:
        return None, None
    return min(forward), statistics.median(forward)


def best_detection(model, results):
    """Return (label, score) for the highest-confidence box, or ('none', None)."""
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return 'none', None
    best = int(boxes.conf.argmax())
    return model.names[int(boxes.cls[best])], float(boxes.conf[best])


class WorldCheckNode(Node):
    """Grab one fresh camera frame and one fresh scan on demand."""

    def __init__(self):
        """Set up the subscriptions and the /cmd_vel publisher."""
        super().__init__('world_check_node')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_image(self, msg):
        self.latest_image = msg

    def _on_scan(self, msg):
        self.latest_scan = msg

    def stop_driving(self):
        """Publish a single zero Twist so teleporting doesn't fight motion."""
        self.cmd_vel_pub.publish(Twist())

    def capture(self):
        """Wait for a fresh image and scan pair, or (None, None) on timeout."""
        self.latest_image = None
        self.latest_scan = None
        deadline = time.monotonic() + CAPTURE_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None and self.latest_scan is not None:
                return self.latest_image, self.latest_scan
        return self.latest_image, self.latest_scan


def format_range(value):
    """Format a range reading for the report, or 'no return' if None."""
    return 'no return' if value is None else f'{value:.2f} m'


def format_detection(label, score):
    """Format a YOLO detection for the report."""
    return label if score is None else f'{label} ({score:.2f})'


def main():
    """Run the sweep over every object and standoff distance."""
    rclpy.init()
    node = WorldCheckNode()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    model = YOLO(MODEL_PATH)
    name_to_id = {name: idx for idx, name in model.names.items()}
    class_ids = [name_to_id[name] for name in TARGET_CLASSES if name in name_to_id]

    header = (
        f'{"object":<17} {"tier":<5} {"dist":>5} {"lidar_min":>10} '
        f'{"lidar_med":>10} {"yolo_all":>20} {"yolo_filtered":>20}'
    )
    print(header)
    print('-' * len(header))

    for object_name, (object_xy, tier) in OBJECTS.items():
        for distance_m in DISTANCES_M:
            pose = pose_for(object_name, object_xy, distance_m)
            if pose is None:
                print(f'{object_name:<17} {tier:<5} {distance_m:>4.1f}m  skipped (too close '
                      'to another object)')
                continue
            rx, ry, yaw = pose

            node.stop_driving()
            teleport('indoor_bot', rx, ry, yaw)
            time.sleep(SETTLE_TIME_S)

            image_msg, scan_msg = node.capture()
            if image_msg is None or scan_msg is None:
                print(f'{object_name:<17} {tier:<5} {distance_m:>4.1f}m  '
                      'no sensor data received')
                continue

            lidar_min, lidar_med = analyze_scan(scan_msg)

            frame = node.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            out_path = RESULTS_DIR / f'{object_name}_{distance_m:.1f}m.png'
            cv2.imwrite(str(out_path), frame)

            results_all = model.predict(
                frame, imgsz=YOLO_IMGSZ, conf=YOLO_CONF, classes=None, verbose=False
            )
            results_filtered = model.predict(
                frame, imgsz=YOLO_IMGSZ, conf=YOLO_CONF, classes=class_ids, verbose=False
            )
            label_all, score_all = best_detection(model, results_all)
            label_filtered, score_filtered = best_detection(model, results_filtered)

            print(
                f'{object_name:<17} {tier:<5} {distance_m:>4.1f}m '
                f'{format_range(lidar_min):>10} {format_range(lidar_med):>10} '
                f'{format_detection(label_all, score_all):>20} '
                f'{format_detection(label_filtered, score_filtered):>20}'
            )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
