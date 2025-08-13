"""
World-frame multi-target tracker for pylons detected as MarkerArray.

This node subscribes to "/detected_pylons" (visualization_msgs/MarkerArray) where
each Marker pose is already in the "map" frame. It maintains a set of tracked
pylon positions (in world/map coordinates) using per-target Kalman filters with
constant-position dynamics, Mahalanobis gating, and Global Nearest Neighbor
assignment (Hungarian algorithm). Tracks are maintained separately for left-side
(red) and right-side (yellow) pylons.

Design choices:
- State: 2D position [x, y]^T in map frame (pylons are static on ground).
- F = I, H = I, Q = q_var * I (small), R = range-dependent diag([sigma^2, sigma^2]).
- Range for R is computed from TF ("map" -> "base_link") if available; else a
  constant sigma is used.
- Gating: chi-square in 2D with threshold ~9.21 (0.99 quantile).
- Data association: Hungarian over gated Mahalanobis distances (global nearest neighbor).
- Track management: new measurements spawn tentative tracks; confirmed after 2 hits;
  tracks are deleted after 3 consecutive misses.
"""
from __future__ import annotations

import math
import time
import numpy as np
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


@dataclass
class TrackerParams:
    """Container for tracker hyperparameters."""
    q_var: float = 1e-4  # process noise variance per axis [m^2]
    sigma_min: float = 0.03  # minimum measurement std [m]
    sigma_slope_per_m: float = 0.02  # slope for range-dependent std [m per m]
    gate_threshold_sq: float = 9.21  # chi-square gate in 2D (approx 0.99 quantile)
    confirm_hits: int = 2  # number of hits to confirm a track
    max_misses: int = 3  # consecutive misses before deletion
    map_frame: str = "map"
    publish_only_confirmed: bool = True
    marker_scale: float = 0.15  # marker diameter [m]
    marker_lifetime_sec: float = 0.0  # 0 means forever
    publish_namespace_left: str = "pylon_map_left"
    publish_namespace_right: str = "pylon_map_right"


@dataclass
class Track2D:
    """Represents a single 2D position track with EKF-like updates (H=I)."""
    track_id: int
    side: str  # "left" or "right"
    x: np.ndarray  # state mean [2]
    P: np.ndarray  # state covariance [2x2]
    hits: int = 0
    misses: int = 0
    confirmed: bool = False
    last_update_time: float = field(default_factory=lambda: time.time())

    def predict(self, q_var: float) -> None:
        """Prediction step with constant position model."""
        Q = np.eye(2) * q_var
        self.P = self.P + Q

    def gating_mahalanobis_sq(self, z: np.ndarray, R: np.ndarray) -> float:
        """Return squared Mahalanobis distance for measurement z with covariance R."""
        S = self.P + R
        v = z - self.x
        try:
            invS = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            invS = np.linalg.pinv(S)
        return float(v.T @ invS @ v)

    def update(self, z: np.ndarray, R: np.ndarray) -> None:
        """Correction step with measurement z and covariance R."""
        S = self.P + R
        try:
            invS = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            invS = np.linalg.pinv(S)
        K = self.P @ invS
        v = z - self.x
        self.x = self.x + K @ v
        I = np.eye(2)
        self.P = (I - K) @ self.P
        self.hits += 1
        self.misses = 0
        self.last_update_time = time.time()
        if not self.confirmed and self.hits >= 2:
            self.confirmed = True

    def miss(self) -> None:
        """Mark a missed update."""
        self.misses += 1
        self.last_update_time = time.time()


class GNNTracker2D:
    """GNN tracker maintaining separate track sets per side (left/right)."""

    def __init__(self, params: TrackerParams, side: str) -> None:
        """
        Initialize tracker.

        Args:
            params: Tracker configuration.
            side: "left" or "right".
        """
        self.params = params
        self.side = side
        self._tracks: List[Track2D] = []
        self._next_id: int = 1

    @property
    def tracks(self) -> List[Track2D]:
        """Return current tracks."""
        return self._tracks

    def step(self, measurements: List[Tuple[np.ndarray, np.ndarray]]) -> None:
        """
        Run one filter+association step for a set of measurements.

        Args:
            measurements: list of (z, R) tuples where z is 2D position and R is 2x2 covariance.
        """
        # Predict all tracks
        for trk in self._tracks:
            trk.predict(self.params.q_var)

        if not self._tracks and not measurements:
            return

        # Build cost matrix of gated Mahalanobis distances
        M = len(self._tracks)
        N = len(measurements)
        if M == 0 and N > 0:
            # Spawn tentative tracks for all measurements
            for z, R in measurements:
                self._spawn_track(z)
            return

        if N == 0 and M > 0:
            # No measurements: all tracks miss
            for trk in self._tracks:
                trk.miss()
            self._prune()
            return

        cost = np.full((M, N), fill_value=1e6, dtype=float)
        for i, trk in enumerate(self._tracks):
            for j, (z, R) in enumerate(measurements):
                d2 = trk.gating_mahalanobis_sq(z, R)
                if d2 <= self.params.gate_threshold_sq:
                    cost[i, j] = d2

        # Solve assignment
        row_ind, col_ind = linear_sum_assignment(cost)

        assigned_tracks = set()
        assigned_meas = set()
        # Apply assignments under gate
        for i, j in zip(row_ind, col_ind):
            if cost[i, j] < 1e5:
                z, R = measurements[j]
                self._tracks[i].update(z, R)
                assigned_tracks.add(i)
                assigned_meas.add(j)

        # Unassigned tracks -> miss
        for i, trk in enumerate(self._tracks):
            if i not in assigned_tracks:
                trk.miss()

        # Unassigned measurements -> spawn tentative tracks
        for j, (z, _) in enumerate(measurements):
            if j not in assigned_meas:
                self._spawn_track(z)

        # Prune stale tracks
        self._prune()

    def _spawn_track(self, z: np.ndarray) -> None:
        """Create a new tentative track initialized at z."""
        P0 = np.eye(2) * 0.25  # initial covariance [m^2], relatively loose
        trk = Track2D(
            track_id=self._next_id,
            side=self.side,
            x=z.copy(),
            P=P0,
            hits=1,
            misses=0,
            confirmed=False,
        )
        self._tracks.append(trk)
        self._next_id += 1

    def _prune(self) -> None:
        """Remove tracks with too many misses."""
        kept: List[Track2D] = []
        for trk in self._tracks:
            if trk.misses > self.params.max_misses:
                continue
            kept.append(trk)
        self._tracks = kept


class MapCreation(Node):
    """ROS 2 node that maintains a world-frame pylon map from detected markers."""

    def __init__(self) -> None:
        """Initialize subscriptions, publishers, TF, and trackers."""
        super().__init__("map_creation_node")

        # Subscriber
        self.sub = self.create_subscription(MarkerArray, "/detected_pylons", self.markers_callback, 10)
        self.ego_odom_sub = self.create_subscription(Odometry, "/ego_odom", self.ego_odom_callback, 10)

        # Publisher
        self.pub = self.create_publisher(MarkerArray, "/pylon_map", 10)

        # Other
        self.car_xy = None
        self.map_frame = "map"

        self.params = TrackerParams()
        self.tracker_left = GNNTracker2D(self.params, side="left")
        self.tracker_right = GNNTracker2D(self.params, side="right")

        self.get_logger().info("Map creation node initialized.")

    def ego_odom_callback(self, msg: Odometry) -> None:
        """Update ego xy position from nav_msgs/Odometry.

        Expects msg.header.frame_id to match the map frame.
        If frames differ, logs a warning and ignores the update.
        """
        self.car_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y),)

    def markers_callback(self, msg: MarkerArray) -> None:
        """Handle incoming detections: build measurements, update trackers, publish map."""
        # Extract measurements per side
        meas_left: List[Tuple[np.ndarray, np.ndarray]] = []
        meas_right: List[Tuple[np.ndarray, np.ndarray]] = []

        car_xy = self.car_xy

        for mk in msg.markers:
            if mk.header.frame_id != self.map_frame:
                # Ignore markers not in map frame
                continue

            side = self._side_from_color(mk.color.r, mk.color.g, mk.color.b)
            if side is None:
                continue

            z = np.array([mk.pose.position.x, mk.pose.position.y], dtype=float)

            dist = None
            if car_xy is not None:
                dx = z[0] - car_xy[0]
                dy = z[1] - car_xy[1]
                dist = math.hypot(dx, dy)

            R = self._measurement_covariance(dist)

            if side == "left":
                meas_left.append((z, R))
            else:
                meas_right.append((z, R))

        # Step trackers
        self.tracker_left.step(meas_left)
        self.tracker_right.step(meas_right)

        # Publish current map
        out = self._make_output_markers()
        self.pub.publish(out)

    def _side_from_color(self, r: float, g: float, b: float) -> Optional[str]:
        """Determine side label from color (red=left, yellow=right)."""
        if r >= 0.9 and g <= 0.1:
            return "left"
        if r >= 0.9 and g >= 0.9:
            return "right"
        return None

    def _measurement_covariance(self, distance_m: Optional[float]) -> np.ndarray:
        """Compute 2x2 measurement covariance R from range; constant if range unknown."""
        if distance_m is None:
            sigma = self.params.sigma_min + 1.0 * self.params.sigma_slope_per_m
        else:
            sigma = self.params.sigma_min + self.params.sigma_slope_per_m * max(0.0, distance_m)
        var = sigma * sigma
        return np.array([[var, 0.0], [0.0, var]], dtype=float)

    def _make_output_markers(self) -> MarkerArray:
        """Build MarkerArray with current tracks."""
        ma = MarkerArray()
        header = Header()
        header.frame_id = self.map_frame

        # Left tracks (red)
        for trk in self.tracker_left.tracks:
            if self.params.publish_only_confirmed and not trk.confirmed:
                continue
            mk = self._track_to_marker(trk, header, ns=self.params.publish_namespace_left, color=(1.0, 0.0, 0.0, 1.0))
            ma.markers.append(mk)

        # Right tracks (yellow)
        for trk in self.tracker_right.tracks:
            if self.params.publish_only_confirmed and not trk.confirmed:
                continue
            mk = self._track_to_marker(trk, header, ns=self.params.publish_namespace_right, color=(1.0, 1.0, 0.0, 1.0))
            ma.markers.append(mk)

        return ma

    def _track_to_marker(self, trk: Track2D, header: Header, ns: str, color: Tuple[float, float, float, float]) -> Marker:
        """Convert a track to a visualization marker."""
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = trk.track_id
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = float(trk.x[0])
        mk.pose.position.y = float(trk.x[1])
        mk.pose.position.z = 0.0
        mk.pose.orientation.x = 0.0
        mk.pose.orientation.y = 0.0
        mk.pose.orientation.z = 0.0
        mk.pose.orientation.w = 1.0
        mk.scale.x = self.params.marker_scale
        mk.scale.y = self.params.marker_scale
        mk.scale.z = self.params.marker_scale
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = color
        if self.params.marker_lifetime_sec > 0.0:
            mk.lifetime = rclpy.duration.Duration(seconds=self.params.marker_lifetime_sec).to_msg()
        return mk


def main(args: Optional[List[str]] = None) -> None:
    """Entry point: initialize ROS 2, create node, and spin."""
    rclpy.init(args=args)
    node = MapCreation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
