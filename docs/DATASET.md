# Dataset bags

Guide for replaying and inspecting the sensor bags recorded with
`scripts/record_dataset.sh` (package `indoor_bot_evaluation`). These bags
exist so perception development does not depend on a running Gazebo
instance.

## Recording

From the workspace root, with the simulation and teleop running:

```
src/indoor_bot_evaluation/scripts/record_dataset.sh [name]
```

(after a colcon build, the installed copy is also available at
`install/indoor_bot_evaluation/share/indoor_bot_evaluation/scripts/record_dataset.sh`;
it is a data file, not a `ros2 run`-able entry point.)

`name` defaults to `indoor_room`. The bag is written to
`data/bags/<name>_<timestamp>/` in the `mcap` storage format. Stop the
recording with `Ctrl+C`; the script then prints the resulting directory
size.

## Playing a bag

```
ros2 bag play data/bags/<name>_<timestamp> --clock --loop
```

`--clock` makes `ros2 bag play` itself publish `/clock` as a live ROS time
source for the duration of playback. The bag already contains a recorded
`/clock` topic from Gazebo, but replaying that as an ordinary topic does not
serve the same purpose: it would not behave as a proper time source across
the discontinuity introduced by `--loop` (time jumping back to the start of
the bag at each iteration), which breaks TF lookups and anything else that
assumes non-decreasing sim time. `--clock` handles looping correctly and is
what lets other nodes' `use_sim_time` clocks track playback.

Any node consuming the bag — including RViz, Foxglove, or a perception
node under development — must be started with `use_sim_time:=true` so it
reads time from `/clock` instead of the wall clock:

```
ros2 run <package> <node> --ros-args -p use_sim_time:=true
```

## Inspecting a bag

```
ros2 bag info data/bags/<name>_<timestamp>
```

This prints the recorded topics, message counts, per-topic types, and the
overall duration and start time, without playing anything back.

## Topics recorded

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | continuous | simulation time from Gazebo |
| `/tf` | `tf2_msgs/TFMessage` | continuous | `odom` -> `base_link` -> sensors |
| `/tf_static` | `tf2_msgs/TFMessage` | once | static sensor transforms |
| `/scan` | `sensor_msgs/LaserScan` | ~9.74 Hz (target 10 Hz) | 180 samples, 5 m range |
| `/odom` | `nav_msgs/Odometry` | 30 Hz | `odom` -> `base_link` |
| `/joint_states` | `sensor_msgs/JointState` | not fixed by the interface contract | wheel joint states; check with `ros2 bag info` |
| `/imu` | `sensor_msgs/Imu` | not fixed by the interface contract | `imu_link` frame; check with `ros2 bag info` |
| `/camera/color/image_raw` | `sensor_msgs/Image` (`rgb8`) | ~9.76 Hz (target 10 Hz) | 320x240 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | ~9.76 Hz (target 10 Hz) | 320x240 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (`32FC1`, metres) | ~9.71 Hz (target 10 Hz) | 320x240 |

Measured rates are from software-rendered runs on the target hardware (no
GPU). See `docs/INTERFACES.md` for the full interface contract and target
rates.
