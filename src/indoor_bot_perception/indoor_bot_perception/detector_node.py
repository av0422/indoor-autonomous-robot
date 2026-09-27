"""YOLO object detector node for the indoor robot."""

import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectorNode(Node):
    """Subscribe to camera images and run object detection on them."""

    def __init__(self):
        """Set up the subscription."""
        super().__init__('detector_node')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos,
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/perception/detections_2d', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/perception/annotated_image', 1
        )
        self.last_annotated = 0.0
        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.model = YOLO('yolo11n.pt')
        self.busy = False
        self.frame_count = 0
        self.latencies = []
        self.get_logger().info('Detector node started')

    def image_callback(self, msg):
        """Run detection on one camera image."""
        if self.busy:
            return
        self.busy = True
        self.frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            start = time.perf_counter()
            results = self.model.predict(
                frame, imgsz=320, conf=self.conf_threshold, verbose=False
            )
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            self.latencies.append(elapsed_ms)

            boxes = results[0].boxes
            detection_msg = self.build_detection_array(boxes, msg.header)
            self.detection_pub.publish(detection_msg)

            now = time.perf_counter()
            if now - self.last_annotated > 0.5:
                annotated = results[0].plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
                self.last_annotated = now

            if len(boxes) > 0:
                names = [self.model.names[int(c)] for c in boxes.cls]
                confs = [round(float(c), 2) for c in boxes.conf]
                self.get_logger().info(f'Detected: {list(zip(names, confs))}')

            if self.frame_count % 100 == 0:
                recent = sorted(self.latencies[-100:])
                p50 = recent[len(recent) // 2]
                p95 = recent[int(len(recent) * 0.95) - 1]
                self.get_logger().info(f'Inference p50={p50:.1f}ms p95={p95:.1f}ms')
        except Exception as exc:
            self.get_logger().error(f'Detection failed: {exc}')
        finally:
            self.busy = False

    def build_detection_array(self, boxes, header):
        """Convert YOLO boxes into a Detection2DArray message."""
        msg = Detection2DArray()
        msg.header = header

        for box in boxes:
            detection = Detection2D()
            detection.header = header

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.model.names[int(box.cls)]
            hypothesis.hypothesis.score = float(box.conf)
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        return msg


def main(args=None):
    """Start the node and spin until interrupted."""
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
