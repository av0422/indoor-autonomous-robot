#!/usr/bin/env python3
r"""
Generate an auto-labelled single-class 'obstacle' dataset from Gazebo.

Ground truth comes from the simulator itself, not from hand annotation or
from a detector: the sim must be launched with `datagen:=true` (see
indoor_bot_gazebo/launch/sim.launch.py), which loads
worlds/indoor_room_datagen.sdf (every object tagged with a unique
gz-sim-label-system label) and adds a pair of boundingbox_camera sensors to
camera_link (indoor_bot_description/urdf/indoor_bot.gazebo.xacro), bridged
to /camera/bounding_box/{visible,full} as vision_msgs/Detection2DArray.

Per episode: every object is teleported to a random floor pose, then the
robot is teleported to several random poses; at each robot pose one camera
frame and the matching bounding boxes are saved. Labels are written in YOLO
format (class 0, normalised cx cy w h) using the *visible* box (the true
on-screen extent, handling occlusion and frame edges), after rejecting
boxes under MIN_BOX_PX on a side or less than MIN_VISIBLE_FRACTION of their
un-occluded (*full*) extent. This script does not start or stop the sim.

Run inside the indoor-bot container, from /ws, with the ROS 2 and venv
environments sourced, sim already launched with datagen:=true:

    python3 src/indoor_bot_evaluation/scripts/datagen.py --seed 0 \\
        --episodes 5 --poses-per-episode 8
"""

import argparse
import math
from pathlib import Path
import random
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# name -> pose to randomise from (x, y ignored, z kept fixed), unique
# per-instance label (matches worlds/indoor_room_datagen.sdf), and tier
# (reporting only). Positions match indoor_room.sdf / indoor_room_datagen.sdf.
OBJECTS = {
    'low_backpack': {'z': 0.06, 'label': 1, 'tier': 'low'},
    'low_suitcase': {'z': 0.07, 'label': 2, 'tier': 'low'},
    'low_sports_ball': {'z': 0.06, 'label': 3, 'tier': 'low'},
    'chair_1': {'z': 0.0, 'label': 4, 'tier': 'tall'},
    'chair_2': {'z': 0.0, 'label': 5, 'tier': 'tall'},
    'chair_3': {'z': 0.0, 'label': 6, 'tier': 'tall'},
    'table_1': {'z': 0.0, 'label': 7, 'tier': 'tall'},
    'potted_plant_1': {'z': 0.0, 'label': 8, 'tier': 'tall'},
}
LABEL_TO_OBJECT = {spec['label']: name for name, spec in OBJECTS.items()}

ROBOT_NAME = 'indoor_bot'
ROBOT_SPAWN_XY = (0.0, 0.0)
ROBOT_Z = 0.05

OBJECT_XY_RANGE_M = 3.2
OBJECT_MIN_CLEARANCE_M = 0.7
OBJECT_SPAWN_CLEARANCE_M = 1.0
ROBOT_XY_RANGE_M = 3.0
ROBOT_OBJECT_CLEARANCE_M = 0.6
MAX_SAMPLE_ATTEMPTS = 500

# Robot pose mix: most poses deliberately face an object close enough to
# produce a usable box (random poses in an 8x8 m room mostly see nothing —
# see docs/DATASET.md), the rest stay fully random for viewpoint diversity.
FACE_OBJECT_FRACTION = 0.70
FACE_LOW_OBJECT_FRACTION = 0.50  # of the facing subset, half target a low object
FACE_DISTANCE_RANGE_M = (0.8, 3.5)
FACE_LOW_DISTANCE_RANGE_M = (0.8, 2.5)
FACE_YAW_JITTER_RAD = math.radians(20.0)
ROOM_HALF_EXTENT_M = 4.0
WALL_MARGIN_M = 0.6
POSE_LIMIT_M = ROOM_HALF_EXTENT_M - WALL_MARGIN_M

SETTLE_TIME_S = 2.0
CAPTURE_TIMEOUT_S = 5.0

MIN_BOX_PX = 6
MIN_VISIBLE_FRACTION = 0.4
SMALL_BOX_PX = 16
MEDIUM_BOX_PX = 48

TRAIN_FRACTION = 0.70
VAL_FRACTION = 0.15

VERIFY_MAX_EPISODES = 20

WORLD_NAME = 'indoor_room'


def teleport(name, x, y, z, yaw):
    """Set an entity's pose in the running world via gz service."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f'name: "{name}", position: {{x:{x:.3f}, y:{y:.3f}, z:{z:.3f}}}, '
        f'orientation: {{z:{qz:.6f}, w:{qw:.6f}}}'
    )
    subprocess.run(
        [
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def sample_object_poses(rng):
    """Return {name: (x, y, yaw)}, respecting clearance from the robot spawn."""
    poses = {}
    for name in OBJECTS:
        x = y = 0.0
        for _attempt in range(MAX_SAMPLE_ATTEMPTS):
            x = rng.uniform(-OBJECT_XY_RANGE_M, OBJECT_XY_RANGE_M)
            y = rng.uniform(-OBJECT_XY_RANGE_M, OBJECT_XY_RANGE_M)
            if math.hypot(x - ROBOT_SPAWN_XY[0], y - ROBOT_SPAWN_XY[1]) < OBJECT_SPAWN_CLEARANCE_M:
                continue
            if any(
                math.hypot(x - px, y - py) < OBJECT_MIN_CLEARANCE_M
                for px, py, _yaw in poses.values()
            ):
                continue
            break
        poses[name] = (x, y, rng.uniform(0.0, 2.0 * math.pi))
    return poses


def _clear_of_objects(x, y, object_poses):
    """Check that (x, y) is at least ROBOT_OBJECT_CLEARANCE_M from every object."""
    return not any(
        math.hypot(x - px, y - py) < ROBOT_OBJECT_CLEARANCE_M
        for px, py, _yaw in object_poses.values()
    )


def _sample_random_pose(rng, object_poses):
    """Return (x, y, yaw) uniform in the room, clear of every object."""
    x = y = 0.0
    for _attempt in range(MAX_SAMPLE_ATTEMPTS):
        x = rng.uniform(-ROBOT_XY_RANGE_M, ROBOT_XY_RANGE_M)
        y = rng.uniform(-ROBOT_XY_RANGE_M, ROBOT_XY_RANGE_M)
        if _clear_of_objects(x, y, object_poses):
            break
    return x, y, rng.uniform(0.0, 2.0 * math.pi)


def _sample_facing_pose(rng, object_poses, target_name, distance_range):
    """
    Return (x, y, yaw) standing distance_range from target_name, facing it.

    Returns None if no clear spot was found in MAX_SAMPLE_ATTEMPTS tries.
    """
    ox, oy, _oyaw = object_poses[target_name]
    for _attempt in range(MAX_SAMPLE_ATTEMPTS):
        distance = rng.uniform(*distance_range)
        approach = rng.uniform(0.0, 2.0 * math.pi)
        x = clamp(ox + distance * math.cos(approach), -POSE_LIMIT_M, POSE_LIMIT_M)
        y = clamp(oy + distance * math.sin(approach), -POSE_LIMIT_M, POSE_LIMIT_M)
        if not _clear_of_objects(x, y, object_poses):
            continue
        base_yaw = math.atan2(oy - y, ox - x)
        yaw = base_yaw + rng.uniform(-FACE_YAW_JITTER_RAD, FACE_YAW_JITTER_RAD)
        return x, y, yaw
    return None


def sample_robot_pose(rng, object_poses):
    """
    Return a robot (x, y, yaw), mostly facing an object, some fully random.

    FACE_OBJECT_FRACTION of poses face a random object within
    FACE_DISTANCE_RANGE_M with FACE_YAW_JITTER_RAD of yaw jitter; half of
    those specifically target a low object at the closer
    FACE_LOW_DISTANCE_RANGE_M, since low objects need to be nearer to clear
    MIN_BOX_PX / MIN_VISIBLE_FRACTION. The rest are fully random. Falls back
    to a random pose if no clear facing spot is found.
    """
    if rng.random() < FACE_OBJECT_FRACTION:
        if rng.random() < FACE_LOW_OBJECT_FRACTION:
            candidates = [n for n in object_poses if OBJECTS[n]['tier'] == 'low']
            distance_range = FACE_LOW_DISTANCE_RANGE_M
        else:
            candidates = list(object_poses)
            distance_range = FACE_DISTANCE_RANGE_M
        target_name = rng.choice(candidates)
        pose = _sample_facing_pose(rng, object_poses, target_name, distance_range)
        if pose is not None:
            return pose
    return _sample_random_pose(rng, object_poses)


def clamp(value, low, high):
    """Clamp value into [low, high]."""
    return max(low, min(high, value))


def bbox_to_xyxy(detection):
    """Convert a Detection2D's centre+size bbox to pixel (x1, y1, x2, y2)."""
    cx = detection.bbox.center.position.x
    cy = detection.bbox.center.position.y
    half_w = detection.bbox.size_x / 2.0
    half_h = detection.bbox.size_y / 2.0
    x1 = clamp(cx - half_w, 0.0, IMAGE_WIDTH)
    y1 = clamp(cy - half_h, 0.0, IMAGE_HEIGHT)
    x2 = clamp(cx + half_w, 0.0, IMAGE_WIDTH)
    y2 = clamp(cy + half_h, 0.0, IMAGE_HEIGHT)
    return x1, y1, x2, y2


def index_by_label(detections):
    """Map each detection's class_id (the gz-sim label, as a string) to itself."""
    return {
        det.results[0].hypothesis.class_id: det
        for det in detections
        if det.results
    }


def build_labels(visible_detections, full_detections):
    """
    Return kept boxes as dicts with a YOLO line, tier, and pixel size.

    A visible box is kept only if it is at least MIN_BOX_PX on each side and
    at least MIN_VISIBLE_FRACTION of its matching full (un-occluded) box's
    area -- whether the missing area is hidden by another object or by the
    frame edge is not distinguished, both make a poor training example.
    """
    full_by_label = index_by_label(full_detections)
    kept = []
    for det in visible_detections:
        if not det.results:
            continue
        class_id = det.results[0].hypothesis.class_id
        full_det = full_by_label.get(class_id)
        if full_det is None:
            continue

        x1, y1, x2, y2 = bbox_to_xyxy(det)
        w_px, h_px = x2 - x1, y2 - y1
        if w_px < MIN_BOX_PX or h_px < MIN_BOX_PX:
            continue

        fx1, fy1, fx2, fy2 = bbox_to_xyxy(full_det)
        full_area = max(fx2 - fx1, 1e-6) * max(fy2 - fy1, 1e-6)
        if (w_px * h_px) / full_area < MIN_VISIBLE_FRACTION:
            continue

        label = int(class_id)
        object_name = LABEL_TO_OBJECT.get(label)
        tier = OBJECTS[object_name]['tier'] if object_name else 'unknown'
        cx_norm = (x1 + x2) / 2.0 / IMAGE_WIDTH
        cy_norm = (y1 + y2) / 2.0 / IMAGE_HEIGHT
        w_norm = w_px / IMAGE_WIDTH
        h_norm = h_px / IMAGE_HEIGHT
        kept.append({
            'line': f'0 {cx_norm:.6f} {cy_norm:.6f} {w_norm:.6f} {h_norm:.6f}',
            'label': label,
            'object': object_name,
            'tier': tier,
            'w_px': w_px,
            'h_px': h_px,
            'xyxy': (x1, y1, x2, y2),
        })
    return kept


def split_episodes(rng, num_episodes):
    """Return {episode_index: 'train'|'val'|'test'}, split by episode."""
    order = list(range(num_episodes))
    rng.shuffle(order)
    n_train = round(num_episodes * TRAIN_FRACTION)
    n_val = round(num_episodes * VAL_FRACTION)
    split = {}
    for i, episode in enumerate(order):
        if i < n_train:
            split[episode] = 'train'
        elif i < n_train + n_val:
            split[episode] = 'val'
        else:
            split[episode] = 'test'
    return split


class DatagenNode(Node):
    """Grab one fresh camera frame and matching bounding-box arrays."""

    def __init__(self):
        """Set up the subscriptions and the /cmd_vel publisher."""
        super().__init__('datagen_node')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_visible = None
        self.latest_full = None
        self.create_subscription(Image, '/camera/color/image_raw', self._on_image, 10)
        self.create_subscription(
            Detection2DArray, '/camera/bounding_box/visible', self._on_visible, 10
        )
        self.create_subscription(
            Detection2DArray, '/camera/bounding_box/full', self._on_full, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_image(self, msg):
        self.latest_image = msg

    def _on_visible(self, msg):
        self.latest_visible = msg

    def _on_full(self, msg):
        self.latest_full = msg

    def stop_driving(self):
        """Publish a single zero Twist so teleporting doesn't fight motion."""
        self.cmd_vel_pub.publish(Twist())

    def capture(self):
        """Wait for a fresh image + box pair, or (None, None, None) on timeout."""
        self.latest_image = None
        self.latest_visible = None
        self.latest_full = None
        deadline = time.monotonic() + CAPTURE_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None and self.latest_visible is not None \
                    and self.latest_full is not None:
                break
        return self.latest_image, self.latest_visible, self.latest_full


def draw_boxes(frame, boxes):
    """Return a copy of frame with each box's xyxy drawn in green."""
    annotated = frame.copy()
    for box in boxes:
        x1, y1, x2, y2 = (int(round(v)) for v in box['xyxy'])
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 220, 0), 1)
    return annotated


def save_contact_sheet(records, out_path, thumb_size=(160, 120)):
    """Tile annotated frames from records into one grid image."""
    if not records:
        return
    cols = math.ceil(math.sqrt(len(records)))
    rows = math.ceil(len(records) / cols)
    thumb_w, thumb_h = thumb_size

    thumbs = []
    for record in records:
        frame = cv2.imread(str(record['image_path']))
        annotated = draw_boxes(frame, record['boxes'])
        thumbs.append(cv2.resize(annotated, (thumb_w, thumb_h)))
    blank = np.zeros((thumb_h, thumb_w, 3), dtype=np.uint8)
    while len(thumbs) < rows * cols:
        thumbs.append(blank)

    grid_rows = [
        np.hstack(thumbs[r * cols:(r + 1) * cols]) for r in range(rows)
    ]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), np.vstack(grid_rows))


def report_stats(records):
    """Print per-split counts, box counts, and a low/tall size histogram."""
    print()
    print('=== Dataset check ===')
    for split in ('train', 'val', 'test'):
        n = sum(1 for r in records if r['split'] == split)
        print(f'{split:>5}: {n} images')

    total_images = len(records)
    total_boxes = sum(len(r['boxes']) for r in records)
    zero_box_images = sum(1 for r in records if not r['boxes'])
    mean_boxes = total_boxes / total_images if total_images else 0.0
    zero_fraction = zero_box_images / total_images if total_images else 0.0
    print(f'total images: {total_images}')
    print(f'total boxes: {total_boxes}')
    print(f'mean boxes/image: {mean_boxes:.2f}')
    print(f'fraction with zero boxes: {zero_fraction:.2%}')

    print(f'box size (max side px): small <{SMALL_BOX_PX}, '
          f'medium <{MEDIUM_BOX_PX}, large >={MEDIUM_BOX_PX}')
    for tier in ('low', 'tall'):
        buckets = {'small': 0, 'medium': 0, 'large': 0}
        for record in records:
            for box in record['boxes']:
                if box['tier'] != tier:
                    continue
                side = max(box['w_px'], box['h_px'])
                if side < SMALL_BOX_PX:
                    buckets['small'] += 1
                elif side < MEDIUM_BOX_PX:
                    buckets['medium'] += 1
                else:
                    buckets['large'] += 1
        print(f'  {tier:>4}: small={buckets["small"]} medium={buckets["medium"]} '
              f'large={buckets["large"]}')


def write_data_yaml(out_dir):
    """Write the Ultralytics dataset config for the single 'obstacle' class."""
    (out_dir / 'data.yaml').write_text(
        f'path: {out_dir.resolve()}\n'
        'train: images/train\n'
        'val: images/val\n'
        'test: images/test\n'
        'names:\n'
        '  0: obstacle\n'
    )


def parse_args():
    """Parse command-line arguments; refuse a large run without --full."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True, help='RNG seed')
    parser.add_argument('--episodes', type=int, default=5)
    parser.add_argument('--poses-per-episode', type=int, default=8)
    parser.add_argument('--out-dir', type=Path, default=Path('data/obstacle_dataset'))
    parser.add_argument('--contact-sheet', type=Path, default=Path('results/datagen_check.png'))
    parser.add_argument('--contact-sheet-limit', type=int, default=40)
    parser.add_argument(
        '--full', action='store_true',
        help=f'Required to run more than {VERIFY_MAX_EPISODES} episodes (the full dataset).',
    )
    args = parser.parse_args()
    if not args.full and args.episodes > VERIFY_MAX_EPISODES:
        parser.error(
            f'--episodes {args.episodes} exceeds the {VERIFY_MAX_EPISODES}-episode '
            'verification cap. Pass --full once you have checked the contact sheet '
            'and are ready to generate the complete dataset.'
        )
    return args


def main():
    """Run the episode sweep and write the dataset, labels, and report."""
    args = parse_args()
    rng = random.Random(args.seed)

    for split in ('train', 'val', 'test'):
        (args.out_dir / 'images' / split).mkdir(parents=True, exist_ok=True)
        (args.out_dir / 'labels' / split).mkdir(parents=True, exist_ok=True)
    write_data_yaml(args.out_dir)

    episode_split = split_episodes(rng, args.episodes)

    rclpy.init()
    node = DatagenNode()
    records = []

    for episode in range(args.episodes):
        split = episode_split[episode]
        object_poses = sample_object_poses(rng)
        for name, (x, y, yaw) in object_poses.items():
            teleport(name, x, y, OBJECTS[name]['z'], yaw)

        for pose_index in range(args.poses_per_episode):
            rx, ry, ryaw = sample_robot_pose(rng, object_poses)
            node.stop_driving()
            teleport(ROBOT_NAME, rx, ry, ROBOT_Z, ryaw)
            time.sleep(SETTLE_TIME_S)

            image_msg, visible_msg, full_msg = node.capture()
            if image_msg is None:
                print(f'ep{episode:04d}_p{pose_index}: no sensor data, skipped')
                continue

            frame = node.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            boxes = build_labels(visible_msg.detections, full_msg.detections)

            stem = f'ep{episode:04d}_p{pose_index}'
            image_path = args.out_dir / 'images' / split / f'{stem}.png'
            label_path = args.out_dir / 'labels' / split / f'{stem}.txt'
            cv2.imwrite(str(image_path), frame)
            label_path.write_text(
                '\n'.join(box['line'] for box in boxes) + ('\n' if boxes else '')
            )

            records.append({'image_path': image_path, 'split': split, 'boxes': boxes})
            print(f'{stem} [{split}] boxes={len(boxes)}')

    node.destroy_node()
    rclpy.shutdown()

    report_stats(records)
    save_contact_sheet(records[:args.contact_sheet_limit], args.contact_sheet)
    print(f'contact sheet: {args.contact_sheet}')


if __name__ == '__main__':
    main()
