"""
Trajectory creation from a streaming cone map with cumulative extension.

This node subscribes to a MarkerArray of cones (red=left, yellow=right) and a PoseStamped
of the ego vehicle in "map". It builds a midline, smooths it, resamples it, and publishes
a PoseArray trajectory. Unlike a stateless planner, this implementation keeps a cumulative
trajectory across updates and only appends a forward extension, preserving the already
published part. It also forbids immediate turns by limiting the heading jump at the
handover between the existing trajectory and the new extension.

Pipeline per update:
1) Split cones by color, pair across sides, compute midpoints.
2) Order midpoints starting near an anchor (previous end if available, else ego).
3) Choose global direction consistent with ego and previous heading.
4) Smooth with Chaikin, then build a forward candidate by projecting the anchor and
   resampling up to horizon.
5) Enforce "no immediate turns" at the join and require a minimal forward gap.
6) Append only the forward extension to the cumulative trajectory.
7) Optionally prune very old points if retain_distance > 0.

Topics:
- Input:
    /pylon_map (visualization_msgs/MarkerArray)
    /pose (geometry_msgs/PoseStamped)
- Output:
    /trajectory (geometry_msgs/PoseArray)

Parameters:
- horizon_distance (float, default 3.0): look-ahead distance in meters.
- sample_spacing (float, default 0.2): spacing between consecutive poses in meters.
- chaikin_iterations (int, default 2): Chaikin smoothing iterations.
- max_turn_deg (float, default 45.0): max allowed heading jump at the join.
- min_extend_dist (float, default sample_spacing): minimal distance from the current end
  to the first new point to begin appending.
- retain_distance (float, default 0.0): if > 0, keep only the last N meters of the
  cumulative trajectory (0 disables pruning).
"""

from typing import List, Tuple

import math
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


def _dist2d(p: np.ndarray, q: np.ndarray) -> float:
    dx = float(p[0] - q[0])
    dy = float(p[1] - q[1])
    return math.hypot(dx, dy)

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.w = math.cos(half)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    return q

def _chaikin(points: np.ndarray, iterations: int) -> np.ndarray:
    """Chaikin corner-cutting for open polylines."""
    if points.shape[0] < 3 or iterations <= 0:
        return points
    out = points.copy()
    for _ in range(iterations):
        new_pts = [out[0]]
        for i in range(out.shape[0] - 1):
            p = out[i]
            q = out[i + 1]
            q1 = 0.75 * p + 0.25 * q
            q2 = 0.25 * p + 0.75 * q
            new_pts.extend([q1, q2])
        new_pts.append(out[-1])
        out = np.vstack(new_pts)
    return out

def _cumulative_lengths(poly: np.ndarray) -> np.ndarray:
    segs = np.linalg.norm(np.diff(poly[:, :2], axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segs)])

def _project_point_onto_polyline(poly: np.ndarray, p: np.ndarray) -> Tuple[int, float, np.ndarray]:
    """
    Project point p (3,) onto polyline 'poly' (N,3).
    Returns (seg_idx, t, proj) with t in [0,1] on segment seg_idx.
    """
    assert poly.shape[0] >= 2
    best_d2 = float("inf")
    best_idx = 0
    best_t = 0.0
    best_proj = poly[0]
    for i in range(poly.shape[0] - 1):
        a = poly[i, :2]
        b = poly[i + 1, :2]
        ab = b - a
        ab2 = float(np.dot(ab, ab))
        if ab2 == 0.0:
            t = 0.0
            proj2 = a
        else:
            t = float(np.dot(p[:2] - a, ab) / ab2)
            t = max(0.0, min(1.0, t))
            proj2 = a + t * ab
        d2 = float(np.dot(p[:2] - proj2, p[:2] - proj2))
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
            best_t = t
            best_proj = np.array([proj2[0], proj2[1], 0.0], dtype=float)
    return best_idx, best_t, best_proj

def _resample_polyline(poly: np.ndarray, ds: float, max_len: float) -> np.ndarray:
    if poly.shape[0] < 2:
        return poly
    s = _cumulative_lengths(poly)
    total = float(min(s[-1], max_len))
    if total <= 0.0:
        return poly[:1]
    n_samples = max(2, int(math.floor(total / ds)) + 1)
    targets = np.linspace(0.0, total, n_samples)
    res = []
    j = 0
    for t in targets:
        while j < len(s) - 2 and s[j + 1] < t:
            j += 1
        s0 = s[j]
        s1 = s[j + 1]
        p0 = poly[j]
        p1 = poly[j + 1]
        if s1 <= s0:
            res.append(p0)
        else:
            a = (t - s0) / (s1 - s0)
            res.append((1.0 - a) * p0 + a * p1)
    return np.vstack(res)

def _order_points_from_seed(points: np.ndarray, seed: np.ndarray) -> np.ndarray:
    """Greedy nearest-neighbor ordering starting from point nearest to seed."""
    if points.shape[0] <= 1:
        return points
    remaining = points.tolist()
    start_idx = int(np.argmin([_dist2d(seed, p) for p in remaining]))
    ordered = [remaining.pop(start_idx)]
    while remaining:
        last = ordered[-1]
        dists = [math.hypot(p[0] - last[0], p[1] - last[1]) for p in remaining]
        i = int(np.argmin(dists))
        ordered.append(remaining.pop(i))
    return np.vstack(ordered)

def _angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


class TrajectoryCreation(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_creation_node")

        # Parameters
        self.declare_parameter("horizon_distance", 3.0)
        self.declare_parameter("sample_spacing", 0.2)
        self.declare_parameter("chaikin_iterations", 2)
        self.declare_parameter("max_turn_deg", 45.0)
        self.declare_parameter("min_extend_dist", 0.0)
        self.declare_parameter("retain_distance", 0.0)

        self.horizon_distance = float(self.get_parameter("horizon_distance").value)
        self.sample_spacing = float(self.get_parameter("sample_spacing").value)
        self.chaikin_iterations = int(self.get_parameter("chaikin_iterations").value)
        self.max_turn_rad = math.radians(float(self.get_parameter("max_turn_deg").value))
        val = float(self.get_parameter("min_extend_dist").value)
        self.min_extend_dist = val if val > 0.0 else self.sample_spacing
        self.retain_distance = float(self.get_parameter("retain_distance").value)

        # Subscriber
        self.pose_sub = self.create_subscription(PoseStamped, "/pose", self.pose_callback, 10)
        self.map_sub = self.create_subscription(MarkerArray, "/pylon_map", self.map_callback, 10)

        # Publisher
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory", 10)

        # Other
        self.current_pose_xy = None  # type: Tuple[float, float]
        self.current_yaw = None  # type: float

        self._traj_pts = None  # type: np.ndarray | None  # shape (N,3)
        self._prev_last_yaw = None  # type: float | None

        self.get_logger().info("Trajectory creation node initialized.")

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        q = msg.pose.orientation
        # Extract yaw from quaternion (ENU)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg: MarkerArray) -> None:
        if self.current_pose_xy is None:
            self.get_logger().warn("No /pose received yet. Skipping trajectory update.")
            return
        if self.current_yaw is None:
            self.get_logger().warn("No heading from /pose yet. Skipping trajectory update.")
            return

        left, right = self._split_cones(msg)
        if left.shape[0] == 0 or right.shape[0] == 0:
            self.get_logger().warn("Left or right cones missing. Skipping update.")
            return

        midpoints = self._pair_and_midpoints(left, right)
        if midpoints.shape[0] == 0:
            self.get_logger().warn("No cone pairs formed. Skipping update.")
            return

        # Anchor: prefer current end; fall back to ego pose on first run.
        if self._traj_pts is not None and self._traj_pts.shape[0] >= 1:
            anchor = self._traj_pts[-1].copy()
        else:
            anchor = np.array([self.current_pose_xy[0], self.current_pose_xy[1], 0.0], dtype=float)

        # Determine previous join yaw (for no-immediate-turn) and the 'last' anchor point.
        if self._traj_pts is not None and self._traj_pts.shape[0] >= 2:
            p_a = self._traj_pts[-2]
            p_b = self._traj_pts[-1]
            prev_yaw = math.atan2(float(p_b[1] - p_a[1]), float(p_b[0] - p_a[0]))
            last = p_b
        else:
            prev_yaw = self.current_yaw
            last = anchor

        # Keep only midpoints that are forward of the current end (with a small tolerance).
        hx, hy = math.cos(prev_yaw), math.sin(prev_yaw)
        rel = midpoints[:, :2] - np.array([[last[0], last[1]]], dtype=float)
        forward_mask = (rel[:, 0] * hx + rel[:, 1] * hy) > -0.2  # allow 20 cm behind
        if forward_mask.any():
            midpoints = midpoints[forward_mask]
        else:
            # No forward points yet; republish and return.
            self.get_logger().debug("No forward midpoints. Republishing current trajectory.")
            if self._traj_pts is not None and self._traj_pts.shape[0] >= 2:
                traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
                self.traj_pub.publish(traj_msg)
            return

        # Need at least 2 forward midpoints to form a segment for projection/resampling.
        if midpoints.shape[0] < 2:
            self.get_logger().debug(
                f"Not enough forward midpoints (n={midpoints.shape[0]}). Holding trajectory."
            )
            if self._traj_pts is not None and self._traj_pts.shape[0] >= 2:
                traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
                self.traj_pub.publish(traj_msg)
            return

        ordered = _order_points_from_seed(midpoints, anchor)

        # Global direction: align with ego and previous heading.
        seg_idx, t, _ = _project_point_onto_polyline(ordered, anchor)
        dir_vec = ordered[seg_idx + 1, :2] - ordered[seg_idx, :2]
        norm = float(np.hypot(dir_vec[0], dir_vec[1]))
        if norm > 0.0:
            tx = float(dir_vec[0]) / norm
            ty = float(dir_vec[1]) / norm
            hx = math.cos(self.current_yaw)
            hy = math.sin(self.current_yaw)
            ego_align = tx * hx + ty * hy
            if self._prev_last_yaw is not None:
                px = math.cos(self._prev_last_yaw)
                py = math.sin(self._prev_last_yaw)
                prev_align = tx * px + ty * py
            else:
                prev_align = 0.0
            if ego_align < 0.0 or prev_align < 0.0:
                ordered = ordered[::-1].copy()
                seg_idx, t, _ = _project_point_onto_polyline(ordered, anchor)

        # Build two orderings: one from 'anchor' and one from a point slightly ahead along heading.
        ahead_dist = max(self.sample_spacing, 0.5)
        hx, hy = math.cos(self.current_yaw), math.sin(self.current_yaw)
        seed_ahead = np.array([anchor[0] + ahead_dist * hx, anchor[1] + ahead_dist * hy, 0.0], dtype=float)

        ordered_a = ordered  # already ordered from 'anchor'
        ordered_b = _order_points_from_seed(midpoints, seed_ahead)

        # Align 'ordered_b' direction in the same way as above (ego + previous heading).
        if ordered_b.shape[0] < 2:
            self.get_logger().debug("ordered_b has <2 points after seeding; using ordered_a only.")
            ordered_b = ordered_a.copy()
        seg_idx_b, t_b, _ = _project_point_onto_polyline(ordered_b, anchor)
        dir_vec_b = ordered_b[seg_idx_b + 1, :2] - ordered_b[seg_idx_b, :2]
        norm_b = float(np.hypot(dir_vec_b[0], dir_vec_b[1]))
        if norm_b > 0.0:
            txb = float(dir_vec_b[0]) / norm_b
            tyb = float(dir_vec_b[1]) / norm_b
            ego_align_b = txb * hx + tyb * hy
            if self._prev_last_yaw is not None:
                px = math.cos(self._prev_last_yaw)
                py = math.sin(self._prev_last_yaw)
                prev_align_b = txb * px + tyb * py
            else:
                prev_align_b = 0.0
            if ego_align_b < 0.0 or prev_align_b < 0.0:
                ordered_b = ordered_b[::-1].copy()
                seg_idx_b, t_b, _ = _project_point_onto_polyline(ordered_b, anchor)

        # For both orderings, smooth, project from 'anchor', and measure forward tail length.
        def _candidate_and_tail(poly: np.ndarray) -> tuple[np.ndarray, float]:
            sm = _chaikin(poly, self.chaikin_iterations)
            si, tt, _ = _project_point_onto_polyline(sm, anchor)
            tmp = np.vstack([(1.0 - tt) * sm[si] + tt * sm[si + 1], sm[si + 1:]])
            s = _cumulative_lengths(tmp)
            tail_len = float(s[-1]) if tmp.shape[0] >= 2 else 0.0
            cand = _resample_polyline(tmp, self.sample_spacing, self.horizon_distance)
            return cand, tail_len

        candidate_a, tail_a = _candidate_and_tail(ordered_a)
        candidate_b, tail_b = _candidate_and_tail(ordered_b)

        # Pick the candidate with the longer tail ahead of the anchor.
        candidate = candidate_a if tail_a >= tail_b else candidate_b

        if candidate.shape[0] < 2:
            # Final fallback: try without smoothing to avoid Chaikin shrinking near the end.
            def _cand_nosmooth(poly: np.ndarray) -> np.ndarray:
                si, tt, _ = _project_point_onto_polyline(poly, anchor)
                tmp = np.vstack([(1.0 - tt) * poly[si] + tt * poly[si + 1], poly[si + 1:]])
                return _resample_polyline(tmp, self.sample_spacing, self.horizon_distance)

            cand_ns = _cand_nosmooth(ordered_a if tail_a >= tail_b else ordered_b)
            if cand_ns.shape[0] < 2:
                self.get_logger().warn(
                    f"Too few points after resampling. mids={midpoints.shape[0]} "
                    f"tailA={tail_a:.2f} tailB={tail_b:.2f} "
                    f"horizon={self.horizon_distance} ds={self.sample_spacing}"
                )
                return
            candidate = cand_ns

        # Project the current end ('last') onto the resampled candidate and start strictly after it.
        # This guarantees forward progress whenever the candidate itself moves forward.
        cand_seg, cand_t, _ = _project_point_onto_polyline(candidate, last)

        # Minimal forward steps corresponding to min_extend_dist.
        min_steps = max(1, int(math.ceil(self.min_extend_dist / self.sample_spacing)))

        # First feasible index: immediately after the projection, plus the minimal step offset.
        i0 = min(candidate.shape[0] - 2, cand_seg + min_steps)

        start_idx = None
        for i in range(i0, candidate.shape[0] - 1):
            c0 = candidate[i]
            # Heading of the join segment is defined by last -> c0
            join_yaw = math.atan2(float(c0[1] - last[1]), float(c0[0] - last[0]))
            yaw_jump = abs(_angle_wrap(join_yaw - prev_yaw))
            if yaw_jump <= self.max_turn_rad:
                start_idx = i
                break

        if start_idx is None:
            # No safe extension yet; keep publishing the cumulative trajectory as-is.
            if self._traj_pts is not None and self._traj_pts.shape[0] >= 2:
                traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
                self.traj_pub.publish(traj_msg)
            else:
                # First run fallback: publish the candidate as initial trajectory.
                self._traj_pts = candidate.copy()
                self._prev_last_yaw = math.atan2(
                    float(self._traj_pts[1, 1] - self._traj_pts[0, 1]),
                    float(self._traj_pts[1, 0] - self._traj_pts[0, 0]),
                )
                traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
                self.traj_pub.publish(traj_msg)
            return

        extension = candidate[start_idx:, :]

        # Drop any prefix of 'extension' that is still too close to 'last'.
        # This ensures we never re-append points we already published.
        cut = 0
        thresh = 0.5 * self.sample_spacing
        while cut < extension.shape[0] and _dist2d(last, extension[cut]) <= thresh:
            cut += 1
        extension = extension[cut:, :]

        # Initialize or append.
        if self._traj_pts is None or self._traj_pts.shape[0] < 1:
            self._traj_pts = extension.copy()
        else:
            if extension.shape[0] == 0:
                # Nothing new yet; just republish current cumulative trajectory.
                traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
                self.traj_pub.publish(traj_msg)
                return
            if _dist2d(self._traj_pts[-1], extension[0]) < 1e-3:
                ext = extension[1:, :]
            else:
                ext = extension
            if ext.shape[0] > 0:
                self._traj_pts = np.vstack([self._traj_pts, ext])

        # Update last yaw for next iteration.
        if self._traj_pts.shape[0] >= 2:
            a = self._traj_pts[-2]
            b = self._traj_pts[-1]
            self._prev_last_yaw = math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))

        # Publish cumulative (optionally pruned) trajectory.
        traj_msg = self._to_pose_array(self._maybe_prune(self._traj_pts))
        self.traj_pub.publish(traj_msg)

    def _maybe_prune(self, pts: np.ndarray) -> np.ndarray:
        """Optionally keep only the last retain_distance meters from the cumulative trajectory."""
        if self.retain_distance <= 0.0 or pts.shape[0] < 2:
            return pts
        s = _cumulative_lengths(pts)
        total = float(s[-1])
        keep_from = 0.0 if total <= self.retain_distance else total - self.retain_distance
        # Find first index with cumulative length >= keep_from
        i0 = int(np.searchsorted(s, keep_from, side="left"))
        return pts[max(0, i0 - 1):, :]

    def _split_cones(self, msg: MarkerArray) -> Tuple[np.ndarray, np.ndarray]:
        """Return left (red) and right (yellow) cones as Nx3 arrays."""
        left = []
        right = []
        for m in msg.markers:
            x = float(m.pose.position.x)
            y = float(m.pose.position.y)
            z = 0.0
            r = float(m.color.r)
            g = float(m.color.g)
            b = float(m.color.b)
            # Red left: (1,0,0), Yellow right: (1,1,0)
            if r >= 0.95 and g < 0.1 and b < 0.1:
                left.append((x, y, z))
            elif r >= 0.95 and g >= 0.95 and b < 0.1:
                right.append((x, y, z))
            else:
                pass
        return np.array(left, dtype=float), np.array(right, dtype=float)

    def _pair_and_midpoints(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Pair cones across sides by nearest neighbor and return unique midpoints."""
        mids: List[Tuple[float, float, float]] = []

        def nearest(point: np.ndarray, cloud: np.ndarray) -> np.ndarray:
            d = np.linalg.norm(cloud[:, :2] - point[:2], axis=1)
            j = int(np.argmin(d))
            return cloud[j]

        for p in left:
            q = nearest(p, right)
            m = 0.5 * (p + q)
            mids.append((float(m[0]), float(m[1]), 0.0))

        for q in right:
            p = nearest(q, left)
            m = 0.5 * (p + q)
            mids.append((float(m[0]), float(m[1]), 0.0))

        # Deduplicate midpoints by rounding to millimeters
        uniq = {}
        for m in mids:
            key = (round(m[0], 3), round(m[1], 3))
            uniq[key] = (m[0], m[1], 0.0)
        arr = np.array(list(uniq.values()), dtype=float)
        return arr

    def _to_pose_array(self, pts: np.ndarray) -> PoseArray:
        header = Header()
        header.frame_id = "map"
        pose_array = PoseArray()
        pose_array.header = header

        # Compute tangents for orientation
        tangents = []
        for i in range(pts.shape[0]):
            if i < pts.shape[0] - 1:
                dx = float(pts[i + 1, 0] - pts[i, 0])
                dy = float(pts[i + 1, 1] - pts[i, 1])
            else:
                dx = float(pts[i, 0] - pts[i - 1, 0])
                dy = float(pts[i, 1] - pts[i - 1, 1])
            tangents.append(math.atan2(dy, dx))

        for i, p in enumerate(pts):
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = 0.0
            pose.orientation = _yaw_to_quat(tangents[i])
            pose_array.poses.append(pose)

        return pose_array


def main(args=None) -> None:
    """Entry point: initialize ROS2, create node, and spin."""
    rclpy.init(args=args)
    node = TrajectoryCreation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
