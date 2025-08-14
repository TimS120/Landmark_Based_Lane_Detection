"""
ROS 2 node for recording an ego vehicle's path, detecting loop closure, smoothing
the closed loop with Chaikin's algorithm, resampling at fixed arc length, and
publishing a single optimized trajectory as geometry_msgs/PoseArray.

Behavior:
- Loop closure criterion relative to the very first pose:
  * Once the vehicle has been farther than 2.0 m from the first pose at any time,
    trigger closure when the current distance to the first pose drops below 0.5 m.
- On closure:
  * Build a closed polyline from the recorded positions.
  * Apply circular Chaikin smoothing with configurable iterations and alpha.
  * Resample the smoothed loop at 0.2 m spacing.
  * Publish as /optimized_trajectory (PoseArray) with z=0.0 and orientation set
    from each pose to the next along the loop.
  * Shutdown the node.

Parameters:
- chaikin_iterations (int, default 10): number of Chaikin passes.
- chaikin_alpha (float, default 0.49): Chaikin corner-cut ratio in (0, 0.5).
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header


# --- Fixed thresholds and resampling step (per specification) ---

MIN_FAR_DISTANCE = 2.0  # [m] must have been at least this far from start once
CLOSE_DISTANCE = 0.5    # [m] then considered closed when nearer than this
RESAMPLE_STEP = 0.2     # [m] arc-length spacing for published poses
FRAME_ID = "map"


def chaikin_circular(poly: np.ndarray, iterations: int, alpha: float) -> np.ndarray:
    """
    Circular Chaikin smoothing on a closed ring (no fixed endpoints).

    Parameters
    ----------
    poly : (N,2) array
        Closed ring WITHOUT duplicated final point; connectivity is cyclic.
    iterations : int
        Number of Chaikin passes (>= 0).
    alpha : float
        Corner-cut ratio in (0, 0.5). Typical 0.25. Higher => stronger smoothing.

    Returns
    -------
    (M,2) array, closed ring WITHOUT duplicated endpoint.
    """
    pts = np.asarray(poly, dtype=float)
    if pts.shape[0] < 3 or iterations <= 0:
        return pts

    a = float(alpha)
    b = 1.0 - a

    for _ in range(int(iterations)):
        n = pts.shape[0]
        q = b * pts + a * np.roll(pts, -1, axis=0)
        r = a * pts + b * np.roll(pts, -1, axis=0)
        out = np.empty((n * 2, 2), dtype=float)
        out[0::2] = q
        out[1::2] = r
        pts = out

    return pts

def close_loop(poly: np.ndarray) -> np.ndarray:
    """
    Ensure a closed polyline by appending the start point to the end if needed.
    """
    if poly.shape[0] == 0:
        return poly
    if np.linalg.norm(poly[0] - poly[-1]) > 0.0:
        return np.vstack([poly, poly[0]])
    return poly

def resample_by_arclength(poly_closed: np.ndarray, step: float) -> np.ndarray:
    """
    Resample a closed polyline at near-uniform arc-length spacing.

    Expects a closed polyline WITH duplicated endpoint.

    Returns
    -------
    (K,2) array WITH duplicated endpoint (loop).
    """
    diffs = np.diff(poly_closed, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]
    if total <= 0.0:
        return poly_closed.copy()

    n = max(int(round(total / max(step, 1e-6))), 3)
    s_new = np.linspace(0.0, total, n + 1)

    out = np.zeros((n + 1, 2), dtype=float)
    j = 0
    for i, si in enumerate(s_new):
        while j < len(s) - 1 and s[j + 1] < si:
            j += 1
        denom = s[j + 1] - s[j]
        t = 0.0 if denom == 0.0 else (si - s[j]) / denom
        out[i] = poly_closed[j] * (1.0 - t) + poly_closed[j + 1] * t

    return out

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert a planar yaw angle [rad] to a geometry_msgs/Quaternion.
    """
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q

def path_to_posearray(loop_xy: np.ndarray, stamp_sec: float) -> PoseArray:
    """
    Convert a closed loop (WITH duplicated endpoint) into a PoseArray.

    Orientation of each pose is the heading from the current point to the next.
    z is set to 0.0 for all poses.

    Parameters
    ----------
    loop_xy : (N,2) array with duplicated endpoint
    stamp_sec : float

    Returns
    -------
    PoseArray in FRAME_ID frame.
    """
    pa = PoseArray()
    pa.header = Header()
    pa.header.frame_id = FRAME_ID
    # Leave stamp at default zero; typically consumers do not require a specific stamp here.

    N = loop_xy.shape[0]
    for i in range(N - 1):
        p0 = loop_xy[i]
        p1 = loop_xy[i + 1]
        d = p1 - p0
        yaw = math.atan2(d[1], d[0]) if (d[0] != 0.0 or d[1] != 0.0) else 0.0

        pose = Pose()
        pose.position.x = float(p0[0])
        pose.position.y = float(p0[1])
        pose.position.z = 0.0
        pose.orientation = yaw_to_quaternion(yaw)
        pa.poses.append(pose)

    return pa


class RouteOptimization(Node):
    """
    ROS 2 node that records /pose, detects loop closure, smooths and resamples the loop,
    publishes /optimized_trajectory once, and shuts down.
    """

    def __init__(self) -> None:
        super().__init__("route_optimization_node")

        # Parameters (only Chaikin as requested)
        self.declare_parameter("chaikin_iterations", 10)
        self.declare_parameter("chaikin_alpha", 0.49)

        self.chaikin_iterations = int(self.get_parameter("chaikin_iterations").value)
        self.chaikin_alpha = float(self.get_parameter("chaikin_alpha").value)

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, "/pose", self.pose_callback, 10
        )
        self.trajectory_sub = self.create_subscription(
            PoseArray, "/trajectory", self.trajectory_callback, 10
        )

        # Publisher
        self.pub_opt = self.create_publisher(PoseArray, "/optimized_trajectory", 10)

        # State
        self.first_xy: np.ndarray | None = None
        self.was_far: bool = False
        self.path_xy: List[Tuple[float, float]] = []
        self.closed_and_published: bool = False

        self.get_logger().info("Route optimization node initialized.")


    def trajectory_callback(self, _msg: PoseArray) -> None:
        """
        Placeholder for future use.
        """
        return

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Record positions and check loop closure. On closure, smooth, resample, publish, and shutdown.
        """
        if self.closed_and_published:
            return

        if msg.header.frame_id and msg.header.frame_id != FRAME_ID:
            # Per specification, everything is in "map"; no TF conversion.
            self.get_logger().warn(
                f"Ignoring pose with frame_id='{msg.header.frame_id}', expected '{FRAME_ID}'."
            )
            return

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        cur = np.array([x, y], dtype=float)

        if self.first_xy is None:
            self.first_xy = cur.copy()
            self.path_xy = [(x, y)]
            self.was_far = False
            return

        self.path_xy.append((x, y))

        # Distance to start
        dist = float(np.linalg.norm(cur - self.first_xy))
        if dist > MIN_FAR_DISTANCE:
            self.was_far = True

        # Closure condition: was farther than MIN_FAR_DISTANCE, now closer than CLOSE_DISTANCE
        if self.was_far and dist < CLOSE_DISTANCE:
            self.get_logger().info("Loop closure detected. Building optimized trajectory.")
            self.build_and_publish()
            self.closed_and_published = True
            # Graceful shutdown shortly after publishing
            self.create_timer(0.2, self._shutdown_once)

    def build_and_publish(self) -> None:
        """
        Build closed loop, smooth via Chaikin, resample, and publish once.
        """
        if len(self.path_xy) < 3 or self.first_xy is None:
            self.get_logger().warn("Not enough points to build a loop.")
            return

        pts = np.array(self.path_xy, dtype=float)

        # Ensure the loop starts at the first pose and ends at the current pose.
        # Close ring WITHOUT duplicate for Chaikin.
        # If final point equals start, drop the duplicated endpoint before Chaikin.
        if np.linalg.norm(pts[0] - pts[-1]) == 0.0:
            ring = pts[:-1]
        else:
            ring = pts

        # Chaikin smoothing on circular ring, then close WITH duplicated endpoint.
        sm_ring = chaikin_circular(ring, iterations=self.chaikin_iterations, alpha=self.chaikin_alpha)
        sm_closed = close_loop(sm_ring)

        # Resample at fixed arc length spacing (keeps duplicated endpoint).
        loop_resampled = resample_by_arclength(sm_closed, RESAMPLE_STEP)

        # Convert to PoseArray with heading from each point to the next.
        pa = path_to_posearray(loop_resampled, stamp_sec=0.0)

        # Publish once.
        self.pub_opt.publish(pa)
        self.get_logger().info(
            f"Published optimized trajectory with {len(pa.poses)} poses to /optimized_trajectory."
        )

    def _shutdown_once(self) -> None:
        """
        One-shot shutdown after publishing.
        """
        self.get_logger().info("Shutting down after publishing optimized trajectory.")
        rclpy.shutdown()


def main(args=None) -> None:
    """
    Entry point: initialize ROS 2, create node, and spin.
    """
    rclpy.init(args=args)
    node = RouteOptimization()
    rclpy.spin(node)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
