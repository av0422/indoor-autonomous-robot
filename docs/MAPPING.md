# Mapping (Milestone 2A)

`indoor_bot_navigation/config/slam_toolbox.yaml` drives `slam_toolbox` in
async mapping mode, building an occupancy grid map of the room from `/scan`
while the robot is driven around. See `docs/INTERFACES.md` for the `/scan`
and TF contract this config assumes.

## Running a mapping session

1. Launch the sim, SLAM, and Foxglove together:

   ```bash
   ros2 launch indoor_bot_bringup mapping.launch.py
   ```

   This runs `indoor_bot_gazebo`'s `sim.launch.py` with `datagen:=false`
   (mapping needs the normal world and bridge topics, not the datagen
   bounding-box cameras), `indoor_bot_navigation`'s `slam.launch.py`, and
   `foxglove_bridge` on port 8765.

2. In a second terminal, drive the robot around the room to build up
   coverage:

   ```bash
   ros2 launch indoor_bot_bringup teleop.launch.py
   ```

   Drive slowly and loop back through already-mapped areas where possible
   so `do_loop_closing` has overlapping scans to correct drift against.

## Inspecting `/map`

- Connect Foxglove Studio to `ws://localhost:8765` and add a Map panel
  subscribed to `/map`, plus a 3D panel with the `map` -> `odom` ->
  `base_footprint` TF tree to watch alignment live.
- From the CLI, confirm the topic is publishing and check its frame:

  ```bash
  ros2 topic hz /map
  ros2 topic echo /map --field header.frame_id
  ```

## Saving the map

Once the room is fully covered and loop closures look clean in Foxglove,
save the map with `nav2_map_server`'s `map_saver_cli`:

```bash
ros2 run nav2_map_server map_saver_cli -f src/indoor_bot_navigation/maps/indoor_room
```

This writes `indoor_room.yaml` and `indoor_room.pgm` into
`indoor_bot_navigation/maps/`, consumed by Nav2's `map_server` in
Milestone 3.
