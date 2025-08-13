"""
ROS2 node to read static pylon positions from a JSON file and publish an accumulated MarkerArray
based on visibility from incoming poses.

Behavior:
- Loads "map1.json" from the share directory of package "landmark_based_lane_detection_pkg".
  JSON schema:
    {
      "pylones": [
        { "x": <float>, "y": <float>, "is_right": <bool or 0/1> },
        ...
      ]
    }
- Subscribes to "/pose" (geometry_msgs/PoseStamped), assuming standard ENU and frame "map".
- For each pose, computes the set of pylons that are visible within a 2D sector:
    * range < 4.0 m (strict)
    * bearing within +-30 degrees (strict) around the pose heading (yaw).
- Adds newly visible pylons to an in-memory accumulated set; avoids duplicates using rounded (x, y).
- Publishes the full accumulated set as a MarkerArray on "/pylon_map" on every pose update.

Notes:
- Frame is "map" throughout; Z is ignored.
- If an already-seen position is observed again with a conflicting "is_right" value, an error is raised.
- Publisher uses transient local QoS so late subscribers receive the latest accumulated map.
"""

import json
import math
import os
from typing import Dict, Tuple, List

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Compute yaw (rotation around +Z) from a quaternion (x, y, z, w) in ENU.

    Returns:
        Yaw in radians in range (-pi, pi].
    """
    # Standard yaw formula for quaternion (x, y, z, w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapCreation(Node):
    """
    Node that accumulates visible pylons from a static map based on incoming poses.
    """

    def __init__(self) -> None:
        super().__init__("simulation_map_creation_node")

        # Parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/pylon_map")
        self.declare_parameter("fov_deg", 30.0)
        self.declare_parameter("range_m", 5.0)
        self.declare_parameter("dedup_round_ndigits", 3)

        # Subscriber
        self.pose_sub = self.create_subscription(PoseStamped, "/pose", self.pose_callback, 10)

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/pylon_map", 10)

        # Other
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.fov_rad = math.radians(
            float(self.get_parameter("fov_deg").get_parameter_value().double_value)
        )
        self.range_m = float(self.get_parameter("range_m").get_parameter_value().double_value)
        self.dedup_ndigits = int(
            self.get_parameter("dedup_round_ndigits").get_parameter_value().integer_value
        )

        pkg_share = os.path.join(get_package_share_directory("landmark_based_lane_detection_pkg"))
        model_path = os.path.join(pkg_share, "map1.json")
        self.static_pylons = self._load_pylons(model_path)

        self.accumulated: Dict[Tuple[float, float], Dict[str, float]] = {}

        self.get_logger().info(
            f"Loaded {len(self.static_pylons)} pylons from '{model_path}'. Publishing on '{self.topic}'."
        )

        self.get_logger().info("Simulation map creation node initialized.")

    def _load_pylons(self, json_path: str) -> List[dict]:
        """
        Load pylons from the given JSON file.

        Returns:
            A list of dicts with keys: "x" (float), "y" (float), "is_right" (bool or 0/1).
        """
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if "pylones" not in data or not isinstance(data["pylones"], list):
            raise ValueError("JSON must contain a 'pylones' list.")

        pylons: List[dict] = []
        for p in data["pylones"]:
            x = float(p["x"])
            y = float(p["y"])
            is_right_val = p.get("is_right")
            if isinstance(is_right_val, bool):
                is_right = is_right_val
            elif isinstance(is_right_val, (int, float)):
                is_right = (is_right_val != 0)
            else:
                raise ValueError("Each pylon must provide 'is_right' as bool or 0/1.")
            pylons.append({"x": x, "y": y, "is_right": is_right})
        return pylons

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Handle incoming poses, update the accumulated map with any newly visible pylons,
        and publish the full accumulated MarkerArray.
        """
        px = msg.pose.position.x
        py = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        yaw = yaw_from_quaternion(qx, qy, qz, qw)

        # Compute newly visible pylons and add them if not already present.
        newly_added = 0
        for p in self.static_pylons:
            if self._is_visible(px, py, yaw, p["x"], p["y"]):
                key = (round(p["x"], self.dedup_ndigits), round(p["y"], self.dedup_ndigits))
                if key in self.accumulated:
                    # Consistency check for is_right
                    existing_is_right = self.accumulated[key]["is_right"]
                    if existing_is_right != p["is_right"]:
                        self.get_logger().error(
                            f"Conflicting 'is_right' for pylon at ~({key[0]}, {key[1]}): "
                            f"existing={existing_is_right}, new={p['is_right']}."
                        )
                        raise RuntimeError("Inconsistent 'is_right' for the same pylon position.")
                    # Already known and consistent: nothing to do.
                else:
                    self.accumulated[key] = {"x": p["x"], "y": p["y"], "is_right": p["is_right"]}
                    newly_added += 1

        # Publish full accumulated map on every pose update (as requested).
        marker_array = self._build_marker_array()
        self.marker_pub.publish(marker_array)

        # if newly_added > 0:
        #     self.get_logger().info(
        #         f"Added {newly_added} new pylons. Accumulated total: {len(self.accumulated)}."
        #     )

    def _is_visible(self, px: float, py: float, yaw: float, tx: float, ty: float) -> bool:
        """
        Check if target point (tx, ty) is visible from pose (px, py, yaw) within
        a sector defined by range < self.range_m and |bearing| < self.fov_rad.

        Returns:
            True if visible, else False.
        """
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        if not (dist < self.range_m):
            return False

        # Bearing of target relative to world
        bearing = math.atan2(dy, dx)

        # Angular difference to heading (wrap to [-pi, pi])
        ang = bearing - yaw
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi

        return abs(ang) < self.fov_rad

    def _build_marker_array(self) -> MarkerArray:
        """
        Build a MarkerArray for the accumulated pylons.
        """
        markers = MarkerArray()
        ns = "accumulated_pylons"

        # Stable ordering for deterministic IDs
        sorted_items = sorted(self.accumulated.items(), key=lambda kv: (kv[0][0], kv[0][1]))
        for idx, (_, p) in enumerate(sorted_items):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = ns
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = p["x"]
            m.pose.position.y = p["y"]
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1

            # Color mapping: left = red, right = yellow
            if p["is_right"]:
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 0.8
            else:
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.8

            markers.markers.append(m)

        return markers


def main(args=None) -> None:
    """Entry point: initialize ROS2, create node, and spin."""
    rclpy.init(args=args)
    node = MapCreation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
