# Indoor Autonomous Robot — Perception-Aware Navigation in ROS 2

A simulated differential-drive robot that maps an indoor space, navigates to a goal
with Nav2, and uses camera-based object detection to avoid obstacles its LiDAR
cannot see.

> **Status: in development (Milestone 0 of 6).** The robot runs in Gazebo with
> working LiDAR, RGB-D camera and IMU, and drives on command. Navigation and
> perception are not implemented yet. Results tables below are intentionally
> empty until the experiments in M6 have actually been run.

## The idea

A 2D LiDAR sees a single horizontal slice of the world. Anything below that plane —
a bag on the floor, a low stool, a cable tray — is invisible to it, and a robot
relying on LiDAR alone will drive straight into it.

This project puts a lightweight object detector on the robot's RGB-D camera,
projects each detection into 3D using the depth image, and publishes the result as a
point cloud that Nav2 consumes as a **second observation source** in its costmap. The
navigation stack then plans around obstacles it otherwise could not perceive.

The claim is testable, so it gets tested: the evaluation is an ablation over
identical trials with perception enabled and disabled.

## Architecture

```
Gazebo Harmonic
  ├─ 2D LiDAR ─────────────┐
  ├─ RGB-D camera ──┐      │
  ├─ IMU            │      │
  └─ contact sensor │      │   (ground-truth collisions, evaluation only)
        ↓ ros_gz_bridge     │
   ┌────┴──────────────┐    │
   │  detector_node    │    │   /perception/detections_2d
   │        ↓          │    │
   │  projector_node   │    │   /perception/detections_3d
   └────┬──────────────┘    │   /perception/obstacle_cloud
        │                   │
        └─────────┬─────────┘
                  ↓
   Nav2 costmap — obstacle layer, two observation sources
                  ↓
   slam_toolbox → map→odom transform
                  ↓
   Nav2: planner_server → controller_server → /cmd_vel
                  ↓
   metrics_logger → CSV
```

The perception and navigation halves communicate only through the topics defined in
[`docs/INTERFACES.md`](docs/INTERFACES.md), which is a frozen contract rather than
documentation written after the fact. That decoupling is what lets the two halves be
developed independently.

## Stack

| Component | Choice |
|---|---|
| ROS 2 | Jazzy Jalisco (LTS) |
| Simulator | Gazebo Harmonic, headless |
| SLAM | `slam_toolbox` |
| Navigation | Nav2 — NavFn planner, DWB controller |
| Detection | Ultralytics YOLO11n, CPU inference |
| Visualisation | Foxglove via `foxglove_bridge` |
| Environment | Docker, arm64 |

Nothing that already exists in the ROS ecosystem is reimplemented. The work is the
robot description, the perception pipeline, the integration between them, and the
evaluation.

## Hardware constraints

Developed on an Apple Silicon MacBook (M2) inside a Docker container. No NVIDIA GPU
and no GPU passthrough to the container, so Gazebo renders camera images in software
and inference runs on CPU. The system is scoped to fit:

- Cameras at 320×240, 10 Hz, rather than 640×480, 15 Hz
- LiDAR at 180 samples, 5 m range
- Measured sensor rates under software rendering: LiDAR 9.74 Hz, colour image
  9.76 Hz, depth image 9.71 Hz (targets 10 Hz)
- A single ~8×8 m room
- DWB controller instead of MPPI, which samples too many trajectories to keep up
- `use_sim_time: true` throughout; time-to-goal is reported in **simulated** seconds
  with the real-time factor logged alongside it, and inference latency in wall-clock
  milliseconds with the hardware named

These are limits worked around deliberately, not defaults left unexamined.

## Results

<!-- Filled in at M6. Empty until the trials have actually been run. -->

Navigation, 20 trials per condition, fixed start/goal pairs with logged seeds:

| Condition | Success rate | Collisions | Time to goal (sim s) | Path length (m) |
|---|---|---|---|---|
| LiDAR only | — | — | — | — |
| LiDAR + perception | — | — | — | — |
| LiDAR + perception, low obstacles | — | — | — | — |

Detection, held-out set of simulated frames:

| Metric | Value |
|---|---|
| mAP@0.5 | — |
| Inference latency, p50 / p95 | — |

Collisions are counted from a Gazebo contact sensor on the robot base rather than
inferred from LiDAR ranges.

## Demo

<!-- Added at M6. -->

## Repository layout

```
src/
  indoor_bot_description/   robot URDF, sensors, frames
  indoor_bot_gazebo/        worlds, models, spawn and bridge launch
  indoor_bot_navigation/    Nav2 and slam_toolbox parameters, maps
  indoor_bot_bringup/       top-level launch files
  indoor_bot_perception/    detector and 3D projection nodes
  indoor_bot_interfaces/    custom messages and actions
  indoor_bot_evaluation/    metrics logging, experiment runner, analysis
docs/
  INTERFACES.md             the contract between the two modules
  ARCHITECTURE.md           design decisions and trade-offs
  SETUP.md                  reproducible environment setup
  DEBUG_LOG.md              problems hit and how they were resolved
```

## Getting started

See [`docs/SETUP.md`](docs/SETUP.md). In short:

```bash
docker compose build
docker compose up -d
docker compose exec ros bash

cd /ws
colcon build --symlink-install
```

Dependency versions are pinned in `docs/requirements.txt`. Two of the pins are
load-bearing: `numpy<2`, because `cv_bridge` is compiled against numpy 1.x, and
`setuptools<80`, which `colcon-core` requires.

## Milestones

| | | Status |
|---|---|---|
| M0 | Workspace, interface contract, CI | done |
| M1 | Robot in simulation, sensors publishing | done |
| M2 | SLAM map, detector publishing detections | planned |
| M3 | Autonomous navigation to a goal, 3D projection | planned |
| M4 | Perception integrated into the costmap | planned |
| M5 | Experiment runner and metrics logging | planned |
| M6 | Experiments, analysis, write-up | planned |

## Licence

AGPL-3.0, required by Ultralytics YOLO. Swapping the detector for
`torchvision`'s SSDLite would allow a permissive licence.