# Detector node performance

Hardware: MacBook Air M4, 16 GB, CPU-only inference in Docker (arm64)
Model: yolo11n.pt, imgsz=320, conf=0.5
Input: rosbag indoor_room_20260926_214338, /camera/color/image_raw at 9.7 Hz

## Inference latency

| Input | p50 | p95 |
|---|---|---|
| Real indoor photographs (11 images) | 122.0 ms | 178.9 ms |
| Simulated camera frames | 25.7 ms | 27.4 ms |

Simulated frames are ~5x faster and far more consistent (2 ms spread vs 57 ms).
Untextured surfaces produce few candidate boxes, so post-processing cost is
low and near-constant.

## Throughput

/perception/detections_2d publishes at 9.69 Hz, matching the camera rate.
The frame-skip guard is not currently triggering; at 27 ms p95 the node
could sustain ~37 Hz. This headroom will shrink once detectable objects
are added to the world.

## Known issue

The simulated room contains no objects from the COCO classes YOLO was
trained on. Remaining detections are false positives ('airplane',
confidence 0.5-0.6) on plain wall surfaces. Detection accuracy cannot be
measured until the world contains recognisable objects.