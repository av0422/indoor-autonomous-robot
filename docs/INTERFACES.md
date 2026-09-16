# Interface Contract

This file is the contract between Module A and Module B. Changes require a
PR reviewed by both.

- **Module A** — robotics/navigation: robot description, Gazebo simulation,
  sensor bridges, Nav2, bringup.
- **Module B** — AI/perception: YOLO-based detection, depth fusion,
  obstacle-cloud generation.

## Module A publishes / Module B consumes

| Topic | Type | Frame | Rate | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `base_scan` | 10 Hz | 180 samples, 5 m range |
| `/camera/color/image_raw` | `sensor_msgs/Image` (`rgb8`) | `camera_color_optical_frame` | 10 Hz | 320x240 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `camera_color_optical_frame` | 10 Hz | |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (`32FC1`, metres) | `camera_depth_optical_frame` | 10 Hz | 320x240 |
| `/odom` | `nav_msgs/Odometry` | `odom` -> `base_link` | 30 Hz | |
| `/tf`, `/tf_static` | | `map` -> `odom` -> `base_link` -> sensors | | |

## Module B publishes / Module A consumes

| Topic | Type | Frame | Rate | Notes |
|---|---|---|---|---|
| `/perception/detections_2d` | `vision_msgs/Detection2DArray` | color optical | >=5 Hz | |
| `/perception/detections_3d` | `vision_msgs/Detection3DArray` | depth optical | >=5 Hz | |
| `/perception/obstacle_cloud` | `sensor_msgs/PointCloud2` | depth optical | >=5 Hz | consumed by Nav2 costmap |
| `/perception/annotated_image` | `sensor_msgs/Image` | - | 2 Hz | debug only |
| `/perception/markers` | `visualization_msgs/MarkerArray` | `map` | 2 Hz | Foxglove |

### Service

- `/perception/set_enabled` (`std_srvs/SetBool`) — toggles perception at
  runtime for the ON/OFF ablation experiment.

## Perception parameters (`detector.yaml`)

| Parameter | Notes |
|---|---|
| `model_path` | |
| `device` | |
| `conf_threshold` | |
| `iou_threshold` | |
| `input_size` | |
| `target_classes` | TBD — owned by Module B |
| `max_depth_m` | |
| `min_points_per_object` | |
| `publish_annotated` | |

## Nav2 costmap obstacle_layer

Two observation sources: `scan` (LaserScan) and `perception` (PointCloud2 on
`/perception/obstacle_cloud`).

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan perception
  scan:
    topic: /scan
    data_type: LaserScan
    marking: true
    clearing: true
  perception:
    topic: /perception/obstacle_cloud
    data_type: PointCloud2
    marking: true
    clearing: false
    min_obstacle_height: 0.02
    max_obstacle_height: 1.0
    obstacle_max_range: 4.0
```

## Hardware constraints

- No GPU, software rendering — cameras run at 320x240 @ 10 Hz.
- DWB controller, not MPPI.
- `use_sim_time` true everywhere.
- Time-to-goal is reported in sim seconds, with real-time factor logged
  alongside.
- Inference latency is reported in wall-clock ms, with the hardware named.

## Known integration gotchas

- (a) Nav2 on Jazzy may emit `Twist` or `TwistStamped` on `/cmd_vel` — set
  this explicitly and match the `ros_gz` bridge configuration.
- (b) Camera optical frame convention (z forward, x right, y down) differs
  from `base_link`.
- (c) `numpy` must stay `<2` because `cv_bridge` is built against numpy 1.x.
- (d) `setuptools` must stay `<80` for colcon.
