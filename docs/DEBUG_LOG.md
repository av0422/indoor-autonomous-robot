# Debug Log

## 2026-09-16 — numpy 2.x breaks cv_bridge

`ultralytics` pulls in `numpy>=2` by default, but `cv_bridge` on ROS 2 Jazzy
is built against numpy 1.x, so a numpy 2.x install breaks image conversion
at runtime.

Fix: pin `numpy<2` (see `docs/requirements.txt`). `ultralytics`' own
warning about missing `opencv-python` (as opposed to
`opencv-python-headless`) is safe to ignore — the container is headless
and `opencv-python-headless` provides the same `cv2` module.
