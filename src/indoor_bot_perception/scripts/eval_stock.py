"""Evaluate stock YOLO against ground-truth obstacle boxes on the test split."""

from pathlib import Path

from ultralytics import YOLO

DATASET = Path('/ws/data/obstacle_dataset')
SPLIT = 'test'
IMGSZ = 320
CONF = 0.25
IOU_MATCH = 0.3
TARGET_CLASSES = {
    'chair', 'potted plant', 'dining table', 'couch', 'backpack',
    'suitcase', 'sports ball', 'bottle', 'person',
}


def load_truth(label_path, width, height):
    """Read a YOLO label file and return boxes as (x1, y1, x2, y2) pixels."""
    boxes = []
    if not label_path.exists():
        return boxes
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cx, cy, w, h = (float(v) for v in parts[1:])
        x1 = (cx - w / 2) * width
        y1 = (cy - h / 2) * height
        x2 = (cx + w / 2) * width
        y2 = (cy + h / 2) * height
        boxes.append((x1, y1, x2, y2))
    return boxes


def iou(a, b):
    """Return intersection over union of two (x1, y1, x2, y2) boxes."""
    ix1 = max(a[0], b[0])
    iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2])
    iy2 = min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def main():
    """Run the stock model on the split and count matches against the truth."""
    model = YOLO('yolo11n.pt')
    image_dir = DATASET / 'images' / SPLIT
    images = sorted(image_dir.glob('*.png')) + sorted(image_dir.glob('*.jpg'))
    tp = fp = fn = 0

    for image_path in images:
        result = model.predict(str(image_path), imgsz=IMGSZ, conf=CONF, verbose=False)[0]
        height, width = result.orig_shape
        label_path = DATASET / 'labels' / SPLIT / (image_path.stem + '.txt')
        truth = load_truth(label_path, width, height)

        preds = []
        for box, cls in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist()):
            if model.names[int(cls)] in TARGET_CLASSES:
                preds.append(tuple(box))

        matched = set()
        for pred in preds:
            best, best_score = None, IOU_MATCH
            for i, true_box in enumerate(truth):
                if i in matched:
                    continue
                score = iou(pred, true_box)
                if score >= best_score:
                    best, best_score = i, score
            if best is None:
                fp += 1
            else:
                matched.add(best)
                tp += 1
        fn += len(truth) - len(matched)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0

    print(f'Split: {SPLIT}  images: {len(images)}')
    print(f'tp={tp}  fp={fp}  fn={fn}')
    print(f'Precision: {precision:.3f}')
    print(f'Recall:    {recall:.3f}')
    print(f'F1:        {f1:.3f}')


if __name__ == '__main__':
    main()
