# Experiments (Milestone 5)

The evaluation harness runs repeated `NavigateToPose` trials against the
simulator and writes one CSV row per trial. Ground-truth collisions come from
a Gazebo contact sensor on the robot base, because the three low objects
(`low_backpack`, `low_suitcase`, `low_sports_ball`) are static and sit below
the 0.20 m LiDAR plane.

## Running

From `/ws` in the container, after `colcon build --symlink-install`:

```bash
ros2 launch indoor_bot_bringup experiment.launch.py \
    condition:=lidar_only trials:=20 seed:=1 output:=results/trials.csv
```

| Argument | Default | Meaning |
|---|---|---|
| `condition` | `unspecified` | Label copied into the CSV. It does not switch anything: toggling perception is done separately (`/perception/set_enabled`) |
| `trials` | `4` | Number of trials |
| `seed` | `0` | Seed for the start-yaw perturbation |
| `pair_index` | `-1` | If 0 or more, every trial uses that pair of `goals.yaml` (0-based); `-1` cycles through the pairs |
| `output` | `results/trials.csv` | CSV file; rows are appended, the header is written once |

The launch starts Gazebo, Nav2 and Foxglove (as `nav.launch.py`) and the
`trial_runner` node, and shuts down when the runner exits. `trial_runner` also
takes `goals_file` (default: the installed `config/goals.yaml`) and `timeout_s`
(default 90 simulated seconds) as parameters.

## Trial procedure

1. Stop the robot, teleport it to the start pose with Gazebo's
   `/world/indoor_room/set_pose`, publish the AMCL `/initialpose`, wait 3
   simulated seconds, clear both costmaps.
2. Send `NavigateToPose` to the goal and record until it finishes or
   `timeout_s` simulated seconds pass (the goal is then cancelled and the
   status is `TIMEOUT`).

Trial `i` uses pair `i mod 5` from `config/goals.yaml` unless `pair_index` is
set. Pairs 0 to 3 are straight lines that each pass within 0.25 m of one low
object; start and goal are on free
floor and at least 0.6 m from every object centre. Pair 4 (`control`) crosses
no object: its line is at least 1.4 m from every object centre and wall face, so
`collision_count` should be 0 on it. Run it alone with `pair_index:=4`.

The seed only perturbs the start yaw by up to +-10 degrees (`random.Random(seed * 1000003 + i)`), so a
trial is reproducible from `(seed, i)`.

## CSV columns

| Column | Unit | Meaning |
|---|---|---|
| `trial` | | Trial index from 0 |
| `seed` | | Run seed |
| `condition` | | Label passed on the command line |
| `success` | 0/1 | 1 if Nav2 reported SUCCEEDED |
| `time_to_goal` | sim s | Simulated time from sending the goal to the result (or to the timeout); also filled for failed trials |
| `path_length_m` | m | Sum of the distances between consecutive `/amcl_pose` messages during the trial. AMCL only publishes after 0.25 m or 0.2 rad of motion (`update_min_d`, `update_min_a`), so this is a coarse estimate |
| `collision_count` | | Distinct contact episodes on `/contact`; contacts less than 0.5 sim s apart are one episode; contacts involving `ground_plane` are ignored |
| `recoveries` | | Largest `number_of_recoveries` in the Nav2 feedback |
| `final_distance_error` | m | Distance from the goal point to the map-frame position of `base_footprint` (TF `map` -> `base_footprint`, i.e. the localisation estimate Nav2 itself uses) when the trial ends |
| `real_time_factor` | | Simulated seconds elapsed divided by wall-clock seconds elapsed over the trial |
| `status` | | `SUCCEEDED`, `ABORTED`, `CANCELED`, `TIMEOUT` or `REJECTED` |
| `pair` | | 0-based index of the `goals.yaml` pair used |

All timing uses the `/clock` from Gazebo (`use_sim_time: true`), so
`time_to_goal` is in simulated seconds and `real_time_factor` says how far
those are from wall-clock seconds.

## Goal tolerance

`nav2_params.yaml` sets `xy_goal_tolerance: 0.15` m and
`yaw_goal_tolerance: 0.25` rad in the DWB `general_goal_checker`. Nav2
reports SUCCEEDED once the robot is within 0.15 m of the goal (by its own
localisation); `final_distance_error` is measured a moment later from the same
localisation, so it can differ slightly, and it includes any AMCL error against
the true pose. `docs/NAVIGATION.md` quotes a default tolerance of about 0.25 m; that
does not match this file.

## Contact sensor

`indoor_bot.gazebo.xacro` adds a `contact` sensor (`base_contact`) on the base
collision (the topic is set inside `<contact>`; at sensor level it is ignored), `indoor_room.sdf` loads `gz-sim-contact-system`, and
`bridge.yaml` bridges the topic to `/contact`
(`ros_gz_interfaces/msg/Contacts`). Only the base collision is monitored, so
wheels and casters touching the floor do not count.
