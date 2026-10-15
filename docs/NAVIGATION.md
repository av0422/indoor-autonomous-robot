# Navigation (Milestone 3)

`indoor_bot_navigation/config/nav2_params.yaml` configures AMCL localizing
against the map saved in `docs/MAPPING.md`, a NavFn global planner, and a
DWB local controller capped at 0.3 m/s / 0.8 rad/s. The costmaps use only
the LiDAR obstacle layer on `/scan` — the perception point-cloud source
from `docs/INTERFACES.md` is added in Milestone 4, not here.

## Running a navigation session

```bash
ros2 launch indoor_bot_bringup nav.launch.py
```

This runs `indoor_bot_gazebo`'s `sim.launch.py` with `datagen:=false`,
`indoor_bot_navigation`'s `navigation.launch.py` (AMCL against
`maps/indoor_room.yaml`, then the Nav2 planner/controller/behavior stack),
and `foxglove_bridge` on port 8765. AMCL's initial pose is set to the
robot's Gazebo spawn point (0, 0, yaw 0), so no manual pose estimate is
needed before sending a goal.

## Sending a goal from Foxglove

1. Connect Foxglove Studio to `ws://localhost:8765`.
2. Add a Map panel subscribed to `/map`, and a 3D panel with the `map` ->
   `odom` -> `base_footprint` TF tree, `/local_costmap/costmap` and
   `/global_costmap/costmap`.
3. Use the "Publish goal" tool (Foxglove's 2D Nav Goal equivalent, publishing
   `geometry_msgs/PoseStamped` on `/goal_pose`) to click a point and
   orientation inside the mapped room.

Alternatively, send a goal from the CLI:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, \
  pose: {position: {x: 1.5, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## What to check

- AMCL's pose estimate (the `map` -> `odom` TF, or the pose array in
  Foxglove) stays near the robot's actual position as it drives — it should
  not jump or drift once a few seconds of `/scan` data have been matched
  against the map.
- The global costmap shows the saved map's walls and furniture; the local
  costmap's obstacle layer updates live as `/scan` changes.
- `ros2 topic echo /cmd_vel` shows `geometry_msgs/msg/Twist` (unstamped),
  not `TwistStamped` — confirms `enable_stamped_cmd_vel: false` took effect
  in both `controller_server` and `velocity_smoother`.
- The robot reaches the goal pose within `xy_goal_tolerance` (0.15 m) and
  `yaw_goal_tolerance` (0.25 rad) without colliding, and the behavior tree
  reports `SUCCEEDED` (`ros2 action list` /
  `/navigate_to_pose/_action/status`).
- `ros2 lifecycle get <node>` on `amcl`, `map_server`, `controller_server`,
  `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`,
  `waypoint_follower`, and `velocity_smoother` all report `active`,
  confirming `autostart: true` configured and activated every lifecycle
  node. `navigation.launch.py` starts these seven Nav2 servers directly
  rather than including nav2_bringup's own `navigation_launch.py`, because
  that file's installed `lifecycle_nodes` list also includes
  `route_server`, `collision_monitor` and `docking_server`, none of which
  have a `nav2_params.yaml` section — `ros2 node list` should show no such
  nodes running.
