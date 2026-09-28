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

## Training dataset

`src/indoor_bot_evaluation/scripts/datagen.py` generates an auto-labelled
image dataset for a custom detector, because stock COCO weights barely
recognise our simulated stand-ins (best scores 0.1-0.3 on the low objects —
see `results/m2_detection.md`). Ground truth comes from the simulator
itself, never from hand annotation or from running YOLO on its own output.

**How it works.** The sim must be launched with `datagen:=true`
(`ros2 launch indoor_bot_gazebo sim.launch.py datagen:=true`), which swaps
in `worlds/indoor_room_datagen.sdf` — the same room as `indoor_room.sdf`,
with every object given a unique `gz-sim-label-system` label — and adds a
pair of `boundingbox_camera` sensors to the robot's `camera_link`
(`indoor_bot_description/urdf/indoor_bot.gazebo.xacro`), co-located with
the RGB-D camera at the same resolution and FOV. This is all inert when
`datagen` is left at its default `false`, so normal simulation and the M6
ablation are unaffected. Per episode, `datagen.py` teleports every object
to a random floor pose (`gz service .../set_pose`) and then the robot to
several random poses, capturing one frame and one label set at each. A
`visible_2d` box gives the true on-screen extent (handling occlusion and
frame edges); a second `full_2d` box for the same instance gives its
un-occluded extent, used only to compute a visibility ratio. Boxes under
6x6 px or less than 40% visible are dropped; poses with no object in view
are kept as negatives rather than discarded.

**Single class.** The robot doesn't need to know *what* an object is, only
that "something is on the floor here" that the LiDAR might miss — so every
object, low and tall alike, trains as one class, `obstacle` (class 0). This
also sidesteps needing enough examples per COCO-style category to train a
multi-class head from a few hundred simulated images.

**Split by episode, not by frame.** All poses from one episode share the
same object layout, so splitting by frame would let near-duplicate views of
the same layout leak between train and test, overstating accuracy. Episodes
are shuffled with the run's `--seed` and assigned whole to train/val/test
(~70/15/15), so a layout only ever appears in one split.

**Known limitation.** Train and test share one room, one set of Fuel-model
textures, and one lighting setup. A detector trained on this dataset is
validated against held-out *poses*, not held-out *environments* — it says
nothing about how the model generalises to a different room, different
object meshes, or real camera images. Treat accuracy numbers from this
dataset as a check that the pipeline works, not as a generalisation claim.
