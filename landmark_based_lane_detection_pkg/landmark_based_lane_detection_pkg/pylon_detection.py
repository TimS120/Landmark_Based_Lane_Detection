import os
import torch
import cv2

import numpy as np
from ultralytics import YOLO

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
import tf2_ros


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to a 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3, dtype=np.float64)
    qx /= n
    qy /= n
    qz /= n
    qw /= n
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rotation_array = np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)

    return rotation_array

def _transform_to_mat(t) -> np.ndarray:
    """Build a 4x4 transform matrix from a geometry_msgs/Transform."""
    R = _quat_to_rot(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[0, 3] = t.translation.x
    T[1, 3] = t.translation.y
    T[2, 3] = t.translation.z
    return T

def _pose_to_mat(p) -> np.ndarray:
    """Build a 4x4 transform matrix from a geometry_msgs/Pose."""
    R = _quat_to_rot(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[0, 3] = p.position.x
    T[1, 3] = p.position.y
    T[2, 3] = p.position.z
    return T


class pylon_detection(Node):
    """
    ROS2 node for detecting pylons using a YOLO model on synchronized
    color and aligned depth images, and publishing their 3D positions as markers.
    """

    def __init__(self):
        super().__init__("pylon_detection_node")

        # Parameters controlling PNG dump of one inferenced frame
        self.declare_parameter("save_one_inference_png", False)
        self.declare_parameter("inference_output_png", "/tmp/yolo_inference.png")
        self.save_one_inference_png = bool(self.get_parameter("save_one_inference_png").get_parameter_value().bool_value)
        self.inference_output_png = str(self.get_parameter("inference_output_png").get_parameter_value().string_value)
        self._saved_png_once = False

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/detected_pylons", 10)

        # Subscribers
        self.color_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera/camera/aligned_depth_to_color/image_raw")
        self.odom_sub = self.create_subscription(Odometry, "/ego_odom", self._odom_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._info_callback,
            10
        )
        self.ts = TimeSynchronizer([self.color_sub, self.depth_sub], 10)
        self.ts.registerCallback(self._image_callback)

        # Other
        self.latest_odom = None

        self.fx = self.fy = self.cx = self.cy = None  # Camera intrinsics (will be use once and then be deleted)

        self.base_frame = "7/base_link"
        self.world_frame = "map"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        pkg_share = os.path.join(get_package_share_directory("landmark_based_lane_detection_pkg"))
        model_path = os.path.join(pkg_share, "best.pt")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        if torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"
        self.get_logger().info(f"Using device: {self.device}")

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

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Store latest odometry of the base frame in the world frame.
        """
        self.latest_odom = msg

    def _get_pylon_depth(self, depth_img: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> float | None:
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

    def _compute_3d(self, u: int, v: int, z: float) -> tuple[float, float, float]:
        """
        Reproject pixel (u, v) with depth z into 3D coordinates in camera frame.
        """
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return (x, y, z)

    def _lookup_transform_mat(self, target_frame: str, source_frame: str, stamp) -> np.ndarray | None:
        """
        Resolve TF from source_frame to target_frame at given time and return as 4x4 matrix.
        """
        if self.tf_buffer.can_transform(target_frame, source_frame, stamp, rclpy.duration.Duration(seconds=0.1)):
            ts = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp)
            return _transform_to_mat(ts.transform)
        return None

    def _image_callback(self, color_msg: Image, depth_msg: Image) -> None:
        """
        Process synchronized color and depth images, detect pylons, and publish markers.
        """
        #print("Synchronized pair received.")
        if any(v is None for v in (self.fx, self.fy, self.cx, self.cy)):
            return

        # Convert images to OpenCV
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # Convert depth to float32 meters before any further use.
        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) * 0.001  # mm -> m
        elif depth_msg.encoding == "32FC1":
            depth = depth.astype(np.float32)  # already meters
        else:
            self.get_logger().warn(f"Unexpected depth encoding: {depth_msg.encoding}. Assuming meters.")
            depth = depth.astype(np.float32)

        # Inference
        results = self.model.predict(
            color,
            conf=0.5,
            verbose=False,
            device=self.device,
            half=True if self.device.startswith("cuda") else False,
        )

        # Prepare markers
        markers = MarkerArray()

        src_frame = color_msg.header.frame_id
        stamp = color_msg.header.stamp

        # camera -> base transform
        T_base_from_cam = self._lookup_transform_mat(self.base_frame, src_frame, stamp)
        base_frame_used = self.base_frame
        if T_base_from_cam is None:
            self.get_logger().warn(
                f"No TF from '{src_frame}' to '{self.base_frame}'. "
                f"Add a static identity between '7/camera_link' and 'camera_link' "
                f"or between '7/base_link' and 'base_link'."
            )

        # world(odom) -> base transform (from /ego_odom)
        T_world_from_base_used = None
        world_frame_used = self.world_frame
        if self.latest_odom is not None:
            world_frame_used = self.latest_odom.header.frame_id or self.world_frame
            T_world_from_child = _pose_to_mat(self.latest_odom.pose.pose)  # odom -> child
            odom_child = self.latest_odom.child_frame_id or "base_link"
            # Treat frames as identical if their basename matches (e.g., "base_link" vs "7/base_link").
            if (
                odom_child == base_frame_used
                or odom_child.split("/")[-1] == base_frame_used.split("/")[-1]
            ):
                T_world_from_base_used = T_world_from_child
            else:
                # Bridge base_link vs 7/base_link via TF
                T_child_from_base = self._lookup_transform_mat(odom_child, base_frame_used, stamp)
                if T_child_from_base is not None:
                    T_world_from_base_used = T_world_from_child @ T_child_from_base
                else:
                    self.get_logger().warn(
                        f"No TF between odom child '{odom_child}' and '{base_frame_used}'. "
                        f"Add a static identity TF (e.g., 7/base_link <-> base_link)."
                    )

        marker_id = 0
        vis_img = color.copy()
        for res in results:
            for box in res.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx_pix = int((x1 + x2) / 2)
                cy_pix = int((y1 + y2) / 2)
                depth_val = self._get_pylon_depth(
                    depth, x1, y1, x2, y2
                )

                z_m = depth_val if depth_val is not None else None
                if depth_val is None:
                    continue

                # 3D in optical camera frame
                x_opt, y_opt, z_opt = self._compute_3d(cx_pix, cy_pix, depth_val)
                p_cam = np.array([x_opt, y_opt, z_opt, 1.0], dtype=np.float64)

                # Creating the markers
                if T_base_from_cam is not None:
                    p_base = T_base_from_cam @ p_cam
                    p_base[2] = 0.0

                    # Odom/world marker (via /ego_odom)
                    if T_world_from_base_used is not None:
                        p_world = T_world_from_base_used @ p_base
                        p_world[2] = 0.0
                        marker_w = Marker()
                        marker_w.header.stamp = color_msg.header.stamp
                        marker_w.header.frame_id = self.world_frame
                        marker_w.ns = "detected_pylons"
                        marker_w.id = marker_id
                        marker_w.type = Marker.SPHERE
                        marker_w.action = Marker.ADD
                        marker_w.pose.position.x = float(p_world[0])
                        marker_w.pose.position.y = float(p_world[1])
                        marker_w.pose.position.z = float(p_world[2])
                        marker_w.pose.orientation.w = 1.0
                        marker_w.scale.x = 0.1
                        marker_w.scale.y = 0.1
                        marker_w.scale.z = 0.1
                        cls_id = int(box.cls[0])
                        if cls_id == 0:  # Left side of the track
                            marker_w.color.r = 1.0
                        else:  # Right side of the track
                            marker_w.color.r = 1.0
                            marker_w.color.g = 1.0
                        marker_w.color.a = 0.8
                        markers.markers.append(marker_w)
                        marker_id += 1

                # Draw visualization overlays
                if self.save_one_inference_png and not self._saved_png_once:
                    #print(self.model.names)
                    cls_id = int(box.cls[0])
                    if cls_id == 0:
                        color = (0, 0, 255)
                    else:
                        color = (0, 255, 255)
                    cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
                    cls_name = self.model.names.get(cls_id, str(cls_id)) if hasattr(self.model, "names") else str(cls_id)
                    conf = float(box.conf[0]) if hasattr(box, "conf") and box.conf is not None else 0.0
                    label = f"{cls_name} {conf:.2f}  z={z_m:.2f}m"
                    cv2.putText(
                        vis_img,
                        label,
                        (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                        cv2.LINE_AA,
                    )

        # Write exactly one PNG if enabled
        if self.save_one_inference_png and not self._saved_png_once:
            cv2.imwrite(self.inference_output_png, vis_img)
            self._saved_png_once = True
            self.get_logger().info(f"Wrote inference PNG to: {self.inference_output_png}")

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
