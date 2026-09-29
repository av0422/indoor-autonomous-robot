# Debug Log

## 2026-09-16 — numpy 2.x breaks cv_bridge

`ultralytics` pulls in `numpy>=2` by default, but `cv_bridge` on ROS 2 Jazzy
is built against numpy 1.x, so a numpy 2.x install breaks image conversion
at runtime.

Fix: pin `numpy<2` (see `docs/requirements.txt`). `ultralytics`' own
warning about missing `opencv-python` (as opposed to
`opencv-python-headless`) is safe to ignore — the container is headless
and `opencv-python-headless` provides the same `cv2` module.

## 2026-09-20 — rosidl_generate_interfaces fails with zero interface files

`indoor_bot_interfaces` called `rosidl_generate_interfaces()` with no
`.msg`/`.srv`/`.action` files, which fails the build ("called without any
interface files") rather than generating an empty package.

Fix: an interfaces package must either define at least one interface or
not call `rosidl_generate_interfaces()` at all. Added
`action/NavigateToObject.action` to give it one.

## 2026-09-21 — ament_copyright fails on launch/display.launch.py

CI failed on `ament_copyright` because `launch/display.launch.py` had no
copyright header.

Fix: decided to disable the copyright linter repo-wide since the licence
is already declared in `LICENSE` and in each package's `package.xml`;
per-file copyright headers are not used in this project. Every other
`ament_lint_auto` linter stays enabled.

## ros2 run fails with ModuleNotFoundError for ultralytics

The wrapper colcon generates in install/ has a hardcoded shebang of
/usr/bin/python3, which does not see packages installed in /ws/.venv.
Activating the venv in the shell does not help, because the wrapper
launches a different interpreter.

Fix: export PYTHONPATH=/ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
before ros2 run, or run the node file directly with python3.

## YOLO false positives on simulated imagery

Running yolo11n on Gazebo camera frames produced detections of 'airplane',
'stop sign' and 'umbrella' in a room containing only grey walls and boxes.
All confidence scores fell between 0.25 and 0.31, i.e. immediately above
the default threshold of 0.25.

Cause: domain gap. The model is trained on photographs; untextured
simulated surfaces do not resemble its training distribution, so it
matches weakly to whatever class is nearest.

Fix: set conf=0.5. Longer term, the simulated world needs models
resembling COCO classes (chairs, potted plants) before detection
accuracy can be measured meaningfully.

## 2026-09-28 — first launch after adding Fuel models will be slow

`indoor_room.sdf` now `<include>`s several models from
`fuel.gazebosim.org` (chairs, a table) for the tall obstacle tier — see
`docs/ARCHITECTURE.md`'s "World layout" section. Gazebo fetches and caches
each `<include>`d Fuel model the first time it is used (into
`~/.gz/fuel` inside the container); every launch after that reuses the
cache and starts at normal speed.

Consequence: the first `ros2 launch indoor_bot_gazebo sim.launch.py` after
this change (or after clearing the `.gz` cache, e.g. a fresh container
volume) will be noticeably slower and requires network access from inside
the container. If the container has no network access, that first launch
will hang or fail to spawn the chair/table models.

## 2026-09-29 — robot pitched nose-down on spawn

Robot pitched 13.5 deg nose-down on spawn. Cause: single rear caster, front
of the body rested on the floor; pitch = asin(0.035/0.15). Effect: LiDAR
scan plane tilted (floor returns at 0.83 m), camera looked 23 deg below
horizontal, earlier tier tests were invalid. Fix: front caster. Check: IMU
orientation y should be near 0 on a fresh spawn.

## Datagen output folder is not cleared between runs

The datagen script writes into data/obstacle_dataset without clearing it.
The two 40-image verify runs left 24 image/label pairs (16 train, 8 val) in
the folders, and the full 150-episode run reused the same episode names
(ep0000-ep0004) with different splits. Result: 1224 files on disk instead
of 1200, and three episode names (ep0000, ep0002, ep0003) appeared in more
than one split. Found by comparing per-split file counts with the
generator's report and by listing episode names across splits. Fixed by
deleting files older than the full run (find ... ! -newermt <date>
-delete) and removing the labels/*.cache files, then re-checking counts
(840/176/184) and split overlap (none). Prevention: clear
data/obstacle_dataset before any full run, or make datagen.py refuse to
start when the output folder is not empty.

## A second gz sim without GZ_PARTITION crashed the running sim

A test launch of a second software-rendered gz sim while the main sim was
running crashed the main instance. Cause: both instances joined the same
transport partition. Fix: set a distinct GZ_PARTITION for any parallel
test instance, or stop the main sim first.
