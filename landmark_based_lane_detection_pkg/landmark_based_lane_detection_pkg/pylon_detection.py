import os

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer


class pylon_detection(Node):
    """
    ROS2 node for detecting cones/pylons using a YOLO model on synchronized
    color and aligned depth images, and publishing their 3D positions as markers.
    """

    def __init__(self):
        super().__init__("pylon_detection_node")

        # Model path from package share
        pkg_share = os.path.join(
            os.getenv('AMENT_PREFIX_PATH', ''),
            'share',
            'landmark_based_lane_detection_pkg'
        )
        model_path = os.path.join(pkg_share, 'resource', 'yolov8n.pt')
        self.model = YOLO(model_path)

        # CV bridge for image conversions
        self.bridge = CvBridge()

        # Camera intrinsics (will be set once)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self._info_callback,
            10
        )

        # Publishers for MarkerArray
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/detected_cones',
            10
        )

        # Subscribers for synchronized color & depth images
        self.color_sub = Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = TimeSynchronizer(
            [self.color_sub, self.depth_sub],
            10
        )
        self.ts.registerCallback(self._image_callback)

        self.get_logger().info("Pylon detection node initialized.")

    def _info_callback(self, msg: CameraInfo) -> None:
        """
        One-time callback to read camera intrinsics.
        """
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )
        # No longer need this subscription
        self.destroy_subscription(self.camera_info_sub)

    def _get_cone_depth(
        self,
        depth_img: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int
    ) -> float | None:
        """
        Sample a lower region of the bounding box, filter out zeros and outliers,
        then return the median Z value (in meters).
        """
        h = y2 - y1
        y_start = int(y2 - 0.2 * h)
        y_end = y2
        x_start = int(x1 + 0.25 * (x2 - x1))
        x_end = int(x2 - 0.25 * (x2 - x1))
        roi = depth_img[y_start:y_end, x_start:x_end]

        vals = roi.flatten()
        vals = vals[vals > 0]
        if vals.size == 0:
            return None

        lo = np.percentile(vals, 10)
        hi = np.percentile(vals, 90)
        filt = vals[(vals >= lo) & (vals <= hi)]
        if filt.size == 0:
            return None

        return float(np.median(filt))

    def _compute_3d(
        self,
        u: int,
        v: int,
        z: float
    ) -> tuple[float, float, float]:
        """
        Reproject pixel (u, v) with depth z into 3D coordinates in camera frame.
        """
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def _image_callback(
        self,
        color_msg: Image,
        depth_msg: Image
    ) -> None:
        """
        Process synchronized color and depth images, detect cones, and publish markers.
        """
        self.get_logger().info("Synchronized pair received.")
        if any(v is None for v in (self.fx, self.fy, self.cx, self.cy)):
            return

        # Convert images to OpenCV
        color = self.bridge.imgmsg_to_cv2(
            color_msg, desired_encoding="bgr8"
        )
        depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        depth = depth.astype(np.float32)

        # YOLO inference
        results = self.model.predict(color, conf=0.5, verbose=False)

        markers = MarkerArray()
        marker_id = 0
        for res in results:
            for box in res.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx_pix = int((x1 + x2) / 2)
                cy_pix = int((y1 + y2) / 2)
                depth_val = self._get_cone_depth(
                    depth, x1, y1, x2, y2
                )
                if depth_val is None:
                    continue

                x, y, z = self._compute_3d(
                    cx_pix, cy_pix, depth_val
                )

                marker = Marker()
                marker.header.stamp = color_msg.header.stamp
                marker.header.frame_id = "map"
                marker.ns = "detected_cones"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                # Color by class (0=red, 1=yellow)
                cls_id = int(box.cls[0])
                if cls_id == 0:
                    marker.color.r = 1.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                marker.color.a = 0.8

                markers.markers.append(marker)
                marker_id += 1

        if markers.markers:
            self.marker_pub.publish(markers)


def main(args=None):
    """Entry point: initialize ROS2, create node, and spin."""
    rclpy.init(args=args)
    pylon_detection_node = pylon_detection()
    rclpy.spin(pylon_detection_node)
    pylon_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
