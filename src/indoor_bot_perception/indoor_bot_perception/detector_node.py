"""YOLO object detector node for the indoor robot."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

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
        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.model = YOLO('yolo11n.pt')
        self.busy = False
        self.frame_count = 0
        self.get_logger().info('Detector node started')

    def image_callback(self, msg):
        """Run detection on one camera image."""
        if self.busy:
            return
        self.busy = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(frame, imgsz=320, conf=self.conf_threshold, verbose=False)
            boxes = results[0].boxes
            if len(boxes) > 0:
                names = [self.model.names[int(c)] for c in boxes.cls]
                confs = [round(float(c), 2) for c in boxes.conf]
                self.get_logger().info(f'Detected: {list(zip(names, confs))}')
        except Exception as exc:
            self.get_logger().error(f'Detection failed: {exc}')
        finally:
            self.busy = False


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