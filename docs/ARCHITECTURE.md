# Architecture

## Overview

## System Diagram

## Robot Description

## Simulation (Gazebo)

### World layout

`indoor_bot_gazebo/worlds/indoor_room.sdf` populates the 8x8 m room with
two tiers of objects, built around a single number: 0.20 m, the exact
height of the 2D LiDAR's scan plane (`scan_height` in
`indoor_bot_description`'s URDF).

- **Tall tier** — chairs, a table, a potted plant (five objects in total,
  mostly Fuel models via `<include>`, spread across the room rather than
  clustered so the robot sees them from multiple angles while driving).
  Every object is at least 0.5 m tall, well above the scan plane, so both
  the LiDAR and the camera detect it. These give the detector realistic
  COCO-class objects to run against — see the false-positive entry in
  `docs/DEBUG_LOG.md`, which is why this world needed real object classes
  in the first place — and make navigation non-trivial, but a LiDAR-only
  robot already avoids them without help.
- **Low tier** — a backpack, a suitcase and a sports ball, all stand-ins
  (see the per-object comments in the SDF for why Fuel models weren't used
  for the first two), placed in open floor gaps between the tall objects.
  Every object is under 0.15 m tall, entirely below the 0.20 m plane, so
  the LiDAR reports clear floor exactly where they sit.

The low tier is what the perception ablation in `README.md`'s M6 actually
measures. With perception disabled, the robot has no sensor evidence these
objects exist; a collision with one — recorded by the Gazebo contact
sensor, not inferred from LiDAR ranges — is the signal that the
camera-based obstacle cloud in the Nav2 costmap adds coverage LiDAR alone
does not have. The tall tier is scenery and detector training ground, not
part of the measurement.

## Sensors and Bridges

## Localization and Mapping

## Navigation (Nav2)

## Perception (YOLO)

## Bringup

## Evaluation and Metrics
