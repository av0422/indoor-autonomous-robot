# indoor-autonomous-robot

An autonomous indoor robot simulation built on ROS 2 Jazzy: Gazebo Harmonic
for simulation, Nav2 for navigation, and a YOLO-based perception pipeline
that feeds detections and an obstacle cloud back into the Nav2 costmap. The
whole stack runs headless in Docker, visualized live over Foxglove.

## Pipeline

```
 sensors (scan, RGB-D, odom, tf)
        |
        v
   perception (YOLO detection + depth fusion)
        |
        v
   costmap (obstacle_layer: scan + perception cloud)
        |
        v
      Nav2 (planner + DWB controller)
        |
        v
     control (/cmd_vel -> ros_gz bridge -> Gazebo)
```

## Modules

| | Module A | Module B |
|---|---|---|
| Focus | Robotics / navigation | AI / perception |
| Owns | Description, Gazebo sim, sensor bridges, Nav2, bringup | YOLO detection, depth fusion, obstacle cloud |
| Packages | `indoor_bot_description`, `indoor_bot_gazebo`, `indoor_bot_navigation`, `indoor_bot_bringup`, `indoor_bot_interfaces` | `indoor_bot_perception`, `indoor_bot_evaluation` |

The interface between the two modules is defined in
[docs/INTERFACES.md](docs/INTERFACES.md) and requires review from both
sides to change.

## Docs

- [docs/SETUP.md](docs/SETUP.md) — Docker-based build and run instructions
- [docs/INTERFACES.md](docs/INTERFACES.md) — Module A / Module B contract
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — system architecture
- [docs/DEBUG_LOG.md](docs/DEBUG_LOG.md) — debugging notes

## CI

![CI](https://img.shields.io/badge/CI-pending-lightgrey)

## Results

_TBD_

## Demo

_TBD_

## Hardware note

Developed on Apple Silicon (M2) in Docker, with Gazebo running headless
(no GPU, no display) and CPU-only inference for the perception pipeline.

## License

AGPL-3.0. This project depends on Ultralytics YOLO, which is licensed
AGPL-3.0, so the whole project is licensed AGPL-3.0 as well. See
[LICENSE](LICENSE).
