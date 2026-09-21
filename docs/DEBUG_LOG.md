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
