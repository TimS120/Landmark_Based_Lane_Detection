"""
ROS 2 node that publishes an initial pose at 10 Hz until a PoseArray trajectory
is received, then simulates driving along that trajectory at constant speed.

Behavior:
- Before any trajectory is received, publishes PoseStamped at (0, 0, 0) with yaw = +pi/2 (heading +Y).
- After receiving a PoseArray on /trajectory, advances along the polyline at speed `velocity` (m/s),
  publishing at 10 Hz on /pose. Heading equals the path tangent; z is fixed to 0.0.
- If a new trajectory arrives, restart from the nearest point on that new path.
- On reaching the end of the path, emit a warning once and hold the final pose.

Parameters:
- velocity (double, default: 1.0)  Constant linear speed in m/s.
- rate_hz (double, default: 10.0)  Publish/update rate in Hz (10 Hz required by the specification).

Topics:
- Subscribes: /trajectory (geometry_msgs/msg/PoseArray)
- Publishes:  /pose       (geometry_msgs/msg/PoseStamped)

Frame:
- header.frame_id = "map" for both input and output.
"""

import math
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert a yaw angle (rad) with zero roll and pitch to a Quaternion.

    Args:
        yaw: Heading angle in radians.

    Returns:
        geometry_msgs.msg.Quaternion representing the rotation.
    """
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q

def segment_projection(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Project point p onto segment ab.

    Args:
        p: Point as (2,) array.
        a: Segment start as (2,) array.
        b: Segment end as (2,) array.

    Returns:
        A tuple (proj, t) where proj is the closest point on the segment,
        and t in [0, 1] is the normalized position along the segment.
    """
    ab = b - a
    ab_len2 = float(np.dot(ab, ab))
    if ab_len2 == 0.0:
        return a.copy(), 0.0
    t = float(np.dot(p - a, ab) / ab_len2)
    t_clamped = max(0.0, min(1.0, t))
    proj = a + t_clamped * ab
    return proj, t_clamped


class SimulateDriving(Node):
    """
    ROS 2 node implementing the specified simulate-driving behavior.
    """

    def __init__(self) -> None:
        super().__init__("simulate_driving_node")

        # Parameters
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("rate_hz", 10.0)
        self.velocity = float(self.get_parameter("velocity").get_parameter_value().double_value)
        self.rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)
        if self.rate_hz <= 0.0:
            self.rate_hz = 10.0
        self.dt = 1.0 / self.rate_hz

        # Subscriber
        self.traj_sub = self.create_subscription(PoseArray, "/trajectory", self.trajectory_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/pose", 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.on_timer)

        # Other
        self.initial_pose_sent = False
        self.following = False
        self.warned_end = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = math.pi / 2.0  # heading +Y

        self.xy: Optional[np.ndarray] = None               # shape (N, 2)
        self.seg_dirs: Optional[np.ndarray] = None         # shape (M, 2) unit vectors per segment
        self.seg_lens: Optional[np.ndarray] = None         # shape (M,)
        self.cum_len: Optional[np.ndarray] = None          # shape (M+1,)
        self.total_len: float = 0.0

        self.s: float = 0.0

        self.get_logger().info("Simulate driving node initialized.")

    def build_polyline(self, poses: List[Tuple[float, float]]) -> None:
        """
        Build polyline and precompute segment directions and cumulative lengths.

        Args:
            poses: List of (x, y) points in order, length >= 2.
        """
        self.xy = np.asarray(poses, dtype=np.float64)
        diffs = self.xy[1:] - self.xy[:-1]
        seg_lens = np.linalg.norm(diffs, axis=1)
        # Replace zero-length segments with epsilon to avoid division by zero in normals.
        safe_lens = np.where(seg_lens == 0.0, 1e-12, seg_lens)
        seg_dirs = diffs / safe_lens[:, None]
        cum = np.zeros(seg_dirs.shape[0] + 1, dtype=np.float64)
        np.cumsum(seg_lens, out=cum[1:])

        self.seg_dirs = seg_dirs
        self.seg_lens = seg_lens
        self.cum_len = cum
        self.total_len = float(cum[-1])
        self.s = 0.0
        self.warned_end = False

    def find_nearest_arc_length(self, px: float, py: float) -> float:
        """
        Find the arc-length position along the path that is nearest to (px, py).

        Args:
            px: Current x.
            py: Current y.

        Returns:
            Arc-length s in [0, total_len].
        """
        assert self.xy is not None and self.seg_dirs is not None and self.cum_len is not None
        p = np.array([px, py], dtype=np.float64)

        best_dist2 = float("inf")
        best_s = 0.0

        for i in range(self.seg_dirs.shape[0]):
            a = self.xy[i]
            b = self.xy[i + 1]
            proj, t = segment_projection(p, a, b)
            dist2 = float(np.dot(p - proj, p - proj))
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_s = float(self.cum_len[i] + t * self.seg_lens[i])

        return max(0.0, min(self.total_len, best_s))

    def pose_at_arc_length(self, s: float) -> Tuple[float, float, float]:
        """
        Compute position and heading at arc-length s.

        Args:
            s: Arc-length along the path.

        Returns:
            (x, y, yaw) at the specified s. z is implicitly 0.0.
        """
        assert self.xy is not None and self.seg_dirs is not None and self.cum_len is not None
        if s <= 0.0:
            dir_vec = self.seg_dirs[0]
            return float(self.xy[0, 0]), float(self.xy[0, 1]), math.atan2(dir_vec[1], dir_vec[0])
        if s >= self.total_len:
            dir_vec = self.seg_dirs[-1]
            return float(self.xy[-1, 0]), float(self.xy[-1, 1]), math.atan2(dir_vec[1], dir_vec[0])

        # Find segment index where cum_len[i] <= s < cum_len[i+1]
        idx = int(np.searchsorted(self.cum_len, s, side="right") - 1)
        idx = max(0, min(idx, self.seg_dirs.shape[0] - 1))
        ds = s - float(self.cum_len[idx])
        t = ds / float(self.seg_lens[idx]) if self.seg_lens[idx] > 0.0 else 0.0
        a = self.xy[idx]
        b = self.xy[idx + 1]
        pos = (1.0 - t) * a + t * b
        dir_vec = self.seg_dirs[idx]
        yaw = math.atan2(dir_vec[1], dir_vec[0])
        return float(pos[0]), float(pos[1]), yaw

    def publish_pose(self, x: float, y: float, yaw: float) -> None:
        """
        Publish PoseStamped on /pose with frame_id "map" and current time.

        Args:
            x: Position x.
            y: Position y.
            yaw: Heading angle in radians.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quaternion(yaw)
        self.pose_pub.publish(msg)

    def trajectory_callback(self, msg: PoseArray) -> None:
        """
        Handle incoming PoseArray trajectory. Builds polyline and sets starting s
        to the nearest point from the current pose, then begins following.
        """
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"Received trajectory with frame_id='{msg.header.frame_id}', expected 'map'. Proceeding as-is."
            )

        if len(msg.poses) < 2:
            self.get_logger().warn("Received trajectory with fewer than 2 points. Ignoring.")
            return

        poses_xy = [(p.position.x, p.position.y) for p in msg.poses]
        self.build_polyline(poses_xy)

        # Start from nearest point to current pose
        self.s = self.find_nearest_arc_length(self.x, self.y)
        self.x, self.y, self.yaw = self.pose_at_arc_length(self.s)
        self.following = True
        self.initial_pose_sent = True  # stop initial publishing
        # self.get_logger().info(
        #     f"New trajectory received with {len(poses_xy)} points. Starting at s={self.s:.3f} m."
        # )

    def on_timer(self) -> None:
        """
        Timer callback executed at rate_hz. Publishes initial pose until a trajectory
        is received, then advances along the trajectory at constant speed.
        """
        if not self.following:
            # Publish initial pose at origin heading +Y
            self.publish_pose(self.x, self.y, self.yaw)
            self.initial_pose_sent = True
            return

        # Following mode
        if self.total_len <= 0.0:
            # Defensive guard; should not happen with valid trajectory.
            self.publish_pose(self.x, self.y, self.yaw)
            return

        step = self.velocity * self.dt
        next_s = self.s + step

        if next_s >= self.total_len:
            # Reached or exceeded the end; hold final pose and warn once.
            if not self.warned_end:
                self.get_logger().warn("End of trajectory reached. Holding final pose until a new trajectory arrives.")
                self.warned_end = True
            self.s = self.total_len
            self.x, self.y, self.yaw = self.pose_at_arc_length(self.s)
            self.publish_pose(self.x, self.y, self.yaw)
            return

        # Advance along the path
        self.s = next_s
        self.x, self.y, self.yaw = self.pose_at_arc_length(self.s)
        self.publish_pose(self.x, self.y, self.yaw)


def main(args=None) -> None:
    """Entry point: initialize ROS 2, create node, and spin."""
    rclpy.init(args=args)
    node = SimulateDriving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
