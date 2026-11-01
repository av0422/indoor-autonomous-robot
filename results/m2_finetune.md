# Detector fine-tuning

Hardware: MacBook Air M4, 16 GB, CPU-only training and inference in Docker (arm64).
Model: yolo11n, single class `obstacle`, imgsz 320.
Training: 840 images, 40 epochs maximum, batch 16, patience 10.
Test split: 184 images, 405 ground-truth boxes. Labels come from Gazebo's
bounding-box camera, not hand annotation.

## Results on the test split

| Model | Method | Precision | Recall | F1 / mAP |
|---|---|---|---|---|
| Stock yolo11n (COCO weights, target classes only) | custom, conf 0.25, IoU 0.3 | 0.952 | 0.198 | F1 0.327 |
| Fine-tuned | custom, conf 0.25, IoU 0.3 | 0.862 | 0.973 | F1 0.914 |
| Fine-tuned | `yolo val` | 0.955 | 0.932 | mAP50 0.972, mAP50-95 0.875 |

Counts for the custom method:

| Model | TP | FP | FN |
|---|---|---|---|
| Stock | 80 | 4 | 325 |
| Fine-tuned | 394 | 63 | 11 |

The custom rows use the same matching rule (a prediction is correct if its IoU
with an unmatched ground-truth box is at least 0.3), so they compare directly.
The `yolo val` row uses Ultralytics' own matching and averaging and must not be
compared with the custom rows.

## Inference latency

Model only (`best.pt`, 100 simulated test frames, 320 px, CPU, MacBook Air M4,
Docker, simulator not running): p50 23.8 ms, p95 24.4 ms, mean 24.1 ms
(about 41 Hz sustainable).

Inside the ROS node with the simulator running the cost is higher because
Gazebo shares the CPU; measure it in the integration step below.

## Reading the results

Fine-tuning raised recall from 0.20 to 0.97: the detector finds 394 of 405
obstacles instead of 80. Precision fell from 0.95 to 0.86, so it reports about
0.34 phantom boxes per image. For a robot that must avoid obstacles a missed
obstacle (a collision) costs more than a phantom one (a detour), so the trade
is acceptable. Whether the phantoms cost navigation time or success is a
question for the perception on/off experiment, not for this table.

## Limitations

- Training and test images come from one room, with the same textures, lighting
  and object set, in different positions. The scores show the model learned this
  simulated world, not that it generalises to other rooms or real obstacles.
- The test split has more empty frames (33 of 184) than train and validation,
  which affects precision.
- The model has not yet been tested inside the navigation loop.
- Single class: the detector says an obstacle is there, not what it is.