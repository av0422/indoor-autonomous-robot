"""Measure YOLO inference latency on a folder of images."""

import statistics
import time
from pathlib import Path

from ultralytics import YOLO

IMAGE_DIR = Path('/ws/data/bench_images')
MODEL = 'yolo11n.pt'
IMGSZ = 320

def main():
    """Run the benchmark and print latency statistics."""
    images = sorted(IMAGE_DIR.glob('*.jpg')) + sorted(IMAGE_DIR.glob('*.png'))
    if not images:
        print(f'No images found in {IMAGE_DIR}')
        return

    print(f'Found {len(images)} images')
    print(f'Loading {MODEL}...')
    model = YOLO(MODEL)

    print('Warming up...')
    for _ in range(3):
        model.predict(str(images[0]), imgsz=IMGSZ, verbose=False)

        latencies = []
    detections = 0

    print('Running benchmark...')
    for image_path in images:
        start = time.perf_counter()
        results = model.predict(str(image_path), imgsz=IMGSZ, verbose=False)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        latencies.append(elapsed_ms)
        if not results:
            print(f'Skipped unreadable image: {image_path.name}')
            latencies.pop()
            continue
        detections += len(results[0].boxes)

        latencies.sort()
    p50 = statistics.median(latencies)
    p95 = latencies[int(len(latencies) * 0.95) - 1]
    mean = statistics.mean(latencies)

    print()
    print(f'Images:          {len(latencies)}')
    print(f'Total detections: {detections}')
    print(f'Mean latency:    {mean:.1f} ms')
    print(f'Median (p50):    {p50:.1f} ms')
    print(f'p95 latency:     {p95:.1f} ms')
    print(f'Max sustainable rate: {1000.0 / p95:.1f} Hz')


if __name__ == '__main__':
    main()