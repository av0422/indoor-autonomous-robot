# YOLO inference latency benchmark

Hardware: MacBook Air M4, 16 GB, CPU-only inference in Docker (arm64)
Model: yolo11n.pt, imgsz=320
Dataset: 11 indoor photographs

| Metric | Value |
|---|---|
| Mean | 121.9 ms |
| Median (p50) | 122.0 ms |
| p95 | 178.9 ms |
| Max sustainable rate | 5.6 Hz |
| Total detections | 32 |

## Implication

The camera publishes at ~9.7 Hz. At 5.6 Hz sustained, the detector cannot
process every frame and must drop older ones. The subscriber will use a
queue depth of 1 so the node always works on the newest available image
rather than falling progressively behind.

This meets the >=5 Hz requirement in docs/INTERFACES.md, but with little margin.

## Outcome (measured after integration)

This prediction did not hold for simulated input. Against the recorded
bag, inference took 25.7 ms p50 / 27.4 ms p95 and the node sustained the
full 9.7 Hz camera rate. Photographs are far more expensive than
untextured simulated frames, which produce few candidate boxes.

The frame-skip mechanism was kept, since inference cost will rise once
the world contains detectable objects. See results/m2_detection.md.