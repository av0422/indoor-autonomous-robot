"""Project 2D obstacle detections into a 3D point cloud using the depth image."""

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray


class ProjectorNode(Node):
    """Turn detection boxes plus depth into points for the Nav2 costmap."""

    def __init__(self):
        """Set up parameters, subscriptions and the cloud publisher."""
        super().__init__('projector_node')
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('min_points_per_object', 5)
        self.declare_parameter('bbox_shrink', 0.15)
        self.declare_parameter('stride', 3)
        self.max_depth = self.get_parameter('max_depth_m').value
        self.min_depth = self.get_parameter('min_depth_m').value
        self.min_points = self.get_parameter('min_points_per_object').value
        self.shrink = self.get_parameter('bbox_shrink').value
        self.stride = self.get_parameter('stride').value

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 1)
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/perception/obstacle_cloud', 10)

        det_sub = message_filters.Subscriber(
            self, Detection2DArray, '/perception/detections_2d')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        self.get_logger().info('Projector node started')

    def info_callback(self, msg):
        """Store the camera intrinsics from the K matrix."""
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def sync_callback(self, detections, depth_msg):
        """Back-project every detection box into 3D points and publish them."""
        if self.fx is None:
            return
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        height, width = depth.shape
        clouds = []

        for det in detections.detections:
            bx = det.bbox.center.position.x
            by = det.bbox.center.position.y
            half_w = det.bbox.size_x * (1.0 - self.shrink) / 2.0
            half_h = det.bbox.size_y * (1.0 - self.shrink) / 2.0
            u0, u1 = max(int(bx - half_w), 0), min(int(bx + half_w) + 1, width)
            v0, v1 = max(int(by - half_h), 0), min(int(by + half_h) + 1, height)

            z = depth[v0:v1:self.stride, u0:u1:self.stride]
            u, v = np.meshgrid(
                np.arange(u0, u1, self.stride), np.arange(v0, v1, self.stride))
            valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
            if valid.sum() < self.min_points:
                continue
            z, u, v = z[valid], u[valid], v[valid]
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            clouds.append(np.stack([x, y, z], axis=1))

        points = np.concatenate(clouds) if clouds else np.empty((0, 3))
        cloud = point_cloud2.create_cloud_xyz32(
            depth_msg.header, points.astype(np.float32).tolist())
        self.cloud_pub.publish(cloud)


def main(args=None):
    """Start the projector node and spin until interrupted."""
    rclpy.init(args=args)
    node = ProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
