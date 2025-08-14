"""
DWA-based local trajectory generator for Formula Student cone courses.

This node subscribes to:
- geometry_msgs/PoseStamped on "/pose" (ego pose in "map")
- visualization_msgs/MarkerArray on "/pylon_map" (cone map in "map")

It publishes:
- geometry_msgs/PoseArray on "/trajectory" (poses in "map")

Model and planner:
- Unicycle kinematics with controls (v, omega).
- Dynamic Window Approach (DWA) samples both v and omega, constant over the roll-out.
- Constraints enforced each cycle:
  - |omega| <= yaw_rate_max_rps, and |omega - last_omega| <= yaw_accel_max_rps2 * dt
  - speed_min_mps <= v <= speed_max_mps, and |v - last_speed| <= accel_max_mps2 * dt
- Obstacles are cone points; no explicit centerline is computed.
- Collision checking uses an inflated circular footprint derived from the
  rectangular footprint plus a safety margin.

Key parameters (declare via ROS params or use defaults):
- fixed_speed_init_mps: initial speed before first plan (subsequent cycles use last_speed).
- speed_min_mps, speed_max_mps: linear speed bounds.
- accel_max_mps2: longitudinal acceleration bound (default 2.0).
- yaw_rate_max_rps, yaw_accel_max_rps2: yaw bounds.
- horizon_s: forward simulation horizon, seconds.
- sim_dt: forward simulation step, seconds.
- sample_count_w: number of omega samples.
- sample_count_v: number of speed samples.
- pose_spacing_m: output pose spacing along the best rollout, meters.
- footprint_length_m, footprint_width_m: vehicle rectangle for footprint.
- safety_margin_m: added margin to the footprint.
- w_clearance, w_balance, w_smooth, w_omega_change, w_progress: cost weights.
- color_tol: tolerance for detecting cone colors.
"""

import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from visualization_msgs.msg import MarkerArray


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to a geometry_msgs/Quaternion with zero roll and pitch."""
    q = Quaternion()
    half = 0.5 * yaw
    q.w = math.cos(half)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    return q

def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi

def integrate_unicycle(x: float, y: float, yaw: float, v: float, omega: float, dt: float) -> Tuple[float, float, float]:
    """Forward integrate unicycle state by dt."""
    if abs(omega) < 1e-6:
        nx = x + v * math.cos(yaw) * dt
        ny = y + v * math.sin(yaw) * dt
        nyaw = yaw
        return nx, ny, nyaw
    R = v / omega
    nyaw = yaw + omega * dt
    nx = x + R * (math.sin(nyaw) - math.sin(yaw))
    ny = y - R * (math.cos(nyaw) - math.cos(yaw))
    nyaw = wrap_angle(nyaw)
    return nx, ny, nyaw

def euclid2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Euclidean distance between 2D points."""
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


class TrajectoryCreation(Node):
    """ROS 2 node that computes a local trajectory using DWA over cones as obstacles."""

    def __init__(self) -> None:
        super().__init__("trajectory_creation_node")

        # Parameters
        self.declare_parameter("fixed_speed_init_mps", 1.0)
        self.declare_parameter("speed_min_mps", 0.1)
        self.declare_parameter("speed_max_mps", 1.5)
        self.declare_parameter("accel_max_mps2", 2.0)

        self.declare_parameter("yaw_rate_max_rps", math.pi)  # 180 deg/s
        self.declare_parameter("yaw_accel_max_rps2", math.pi)  # 180 deg/s^2

        self.declare_parameter("horizon_s", 3.0)
        self.declare_parameter("sim_dt", 0.1)
        self.declare_parameter("sample_count_w", 31)
        self.declare_parameter("sample_count_v", 5)
        self.declare_parameter("pose_spacing_m", 0.2)

        self.declare_parameter("footprint_length_m", 0.5)
        self.declare_parameter("footprint_width_m", 0.35)
        self.declare_parameter("safety_margin_m", 0.05)

        self.declare_parameter("w_clearance", 1.0)
        self.declare_parameter("w_balance", 0.4)
        self.declare_parameter("w_smooth", 0.2)
        self.declare_parameter("w_omega_change", 0.2)
        self.declare_parameter("w_progress", 0.5)

        self.declare_parameter("w_heading", 0.8)  # penalize end heading change
        self.declare_parameter("w_forward", 0.8)  # reward forward progress along current yaw

        self.declare_parameter("color_tol", 0.15)

        # Subscriber
        self.create_subscription(PoseStamped, "/pose", self.pose_cb, 10)
        self.create_subscription(MarkerArray, "/pylon_map", self.map_cb, 10)

        # Publisher
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory", 1)

        # Other
        self.current_pose: Optional[PoseStamped] = None
        self.last_omega: float = 0.0
        self.last_speed: float = float(self.get_parameter("fixed_speed_init_mps").value)

        L = float(self.get_parameter("footprint_length_m").value)
        W = float(self.get_parameter("footprint_width_m").value)
        margin = float(self.get_parameter("safety_margin_m").value)
        self.robot_radius = math.hypot(L * 0.5, W * 0.5) + margin

        self.get_logger().info("Trajectory creation node initialized.")

    def pose_cb(self, msg: PoseStamped) -> None:
        """Store the latest ego pose."""
        self.current_pose = msg

    def map_cb(self, msg: MarkerArray) -> None:
        """On new cone map: run DWA and publish trajectory if pose is known."""
        if self.current_pose is None:
            self.get_logger().warn("Received /pylon_map but no /pose yet; skipping.")
            return

        # Extract cone points and split by side using color.
        color_tol = float(self.get_parameter("color_tol").value)
        left_cones: List[Tuple[float, float]] = []   # red
        right_cones: List[Tuple[float, float]] = []  # yellow

        for m in msg.markers:
            x = m.pose.position.x
            y = m.pose.position.y
            r = float(m.color.r)
            g = float(m.color.g)
            b = float(m.color.b)

            if abs(r - 1.0) <= color_tol and abs(g - 0.0) <= color_tol and abs(b - 0.0) <= color_tol:
                left_cones.append((x, y))
            elif abs(r - 1.0) <= color_tol and abs(g - 1.0) <= color_tol and abs(b - 0.0) <= color_tol:
                right_cones.append((x, y))
            else:
                # Unknown color; treat as generic obstacle by adding to both lists.
                left_cones.append((x, y))
                right_cones.append((x, y))

        if len(left_cones) == 0 or len(right_cones) == 0:
            self.get_logger().warn("Only one cone side or sparse map; balance cost disabled for this cycle.")

        traj = self.plan_dwa(left_cones, right_cones)

        if traj is not None:
            self.traj_pub.publish(traj)
        else:
            self.get_logger().warn("DWA failed to find a collision-free rollout; publishing skipped.")

    def plan_dwa(self, left_cones: List[Tuple[float, float]], right_cones: List[Tuple[float, float]]) -> Optional[PoseArray]:
        """Run one DWA cycle and return the best PoseArray or None if infeasible."""
        # Parameters.
        v_min = float(self.get_parameter("speed_min_mps").value)
        v_max = float(self.get_parameter("speed_max_mps").value)
        a_max = float(self.get_parameter("accel_max_mps2").value)

        w_max = float(self.get_parameter("yaw_rate_max_rps").value)
        w_acc = float(self.get_parameter("yaw_accel_max_rps2").value)

        T = float(self.get_parameter("horizon_s").value)
        dt = float(self.get_parameter("sim_dt").value)
        samples_w = int(self.get_parameter("sample_count_w").value)
        samples_v = int(self.get_parameter("sample_count_v").value)
        spacing = float(self.get_parameter("pose_spacing_m").value)

        w_clearance = float(self.get_parameter("w_clearance").value)
        w_balance = float(self.get_parameter("w_balance").value)
        w_smooth = float(self.get_parameter("w_smooth").value)
        w_wchange = float(self.get_parameter("w_omega_change").value)
        w_progress = float(self.get_parameter("w_progress").value)
        
        w_heading = float(self.get_parameter("w_heading").value)
        w_forward = float(self.get_parameter("w_forward").value)

        # Ego state.
        pose = self.current_pose.pose
        x = pose.position.x
        y = pose.position.y

        # Extract yaw from quaternion.
        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Dynamic window for omega and v.
        w_min_win = max(-w_max, self.last_omega - w_acc * dt)
        w_max_win = min(w_max, self.last_omega + w_acc * dt)

        v_min_win = max(v_min, self.last_speed - a_max * dt)
        v_max_win = min(v_max, self.last_speed + a_max * dt)

        if samples_w < 2:
            samples_w = 2
        if samples_v < 1:
            samples_v = 1

        candidate_omegas = [w_min_win + i * (w_max_win - w_min_win) / (samples_w - 1) for i in range(samples_w)]
        candidate_speeds: List[float]
        if samples_v == 1:
            candidate_speeds = [max(v_min, min(v_max, self.last_speed))]
        else:
            candidate_speeds = [v_min_win + i * (v_max_win - v_min_win) / (samples_v - 1) for i in range(samples_v)]

        # Obstacles for clearance queries.
        obstacles = left_cones + right_cones

        best_cost = float("inf")
        best_path: List[Tuple[float, float, float]] = []
        best_omega = self.last_omega
        best_speed = self.last_speed

        for omega in candidate_omegas:
            for v in candidate_speeds:
                path_xyyaw: List[Tuple[float, float, float]] = []
                sx, sy, syaw = x, y, yaw
                min_clear = float("inf")
                min_left = float("inf")
                min_right = float("inf")
                collision = False
                steps = max(1, int(T / dt))
                path_len = 0.0

                for _ in range(steps):
                    nx, ny, nyaw = integrate_unicycle(sx, sy, syaw, v, omega, dt)
                    path_xyyaw.append((nx, ny, nyaw))
                    seg = euclid2((nx, ny), (sx, sy))
                    path_len += seg

                    # Clearance check.
                    for ox, oy in obstacles:
                        d = euclid2((nx, ny), (ox, oy))
                        if d <= self.robot_radius:
                            collision = True
                            break
                        if d < min_clear:
                            min_clear = d
                    if collision:
                        break

                    if left_cones:
                        d_left = min(euclid2((nx, ny), c) for c in left_cones)
                        if d_left < min_left:
                            min_left = d_left
                    if right_cones:
                        d_right = min(euclid2((nx, ny), c) for c in right_cones)
                        if d_right < min_right:
                            min_right = d_right

                    sx, sy, syaw = nx, ny, nyaw

                if collision or not path_xyyaw:
                    continue

                # Terminal state after rollout
                end_x, end_y, end_yaw = sx, sy, syaw

                # Costs.
                clear_cost = 1.0 / max(min_clear - self.robot_radius, 1e-3)
                if (min_left < float("inf")) and (min_right < float("inf")):
                    balance_cost = abs(min_left - min_right)
                else:
                    balance_cost = 0.0
                smooth_cost = abs(omega)
                wchange_cost = abs(omega - self.last_omega)
                progress_cost = -path_len  # prefer longer progress along the horizon

                # NEW: penalize turning back (terminal heading change)
                heading_change = abs(wrap_angle(end_yaw - yaw))
                heading_cost = heading_change

                # NEW: reward forward displacement along current yaw (discourage U-turn arcs)
                dx, dy = (end_x - x), (end_y - y)
                forward_proj = dx * math.cos(yaw) + dy * math.sin(yaw)
                forward_cost = -forward_proj  # more forward (>=0) -> lower cost

                total_cost = (
                    w_clearance * clear_cost
                    + w_balance * balance_cost
                    + w_smooth * smooth_cost
                    + w_wchange * wchange_cost
                    + w_progress * progress_cost
                    + w_heading * heading_cost
                    + w_forward * forward_cost
                )

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_path = path_xyyaw
                    best_omega = omega
                    best_speed = v

        if not best_path:
            return None

        # Update last controls for next cycle.
        self.last_omega = best_omega
        self.last_speed = best_speed

        # Convert the best path to a PoseArray with approximately uniform spacing.
        traj = PoseArray()
        traj.header.frame_id = "map"
        traj.header.stamp = self.get_clock().now().to_msg()

        poses: List[Pose] = []
        if spacing <= 0.0:
            spacing = 0.2

        # Include the start pose.
        start_pose = Pose()
        start_pose.position.x = x
        start_pose.position.y = y
        start_pose.position.z = 0.0
        start_pose.orientation = yaw_to_quat(yaw)
        poses.append(start_pose)

        last_added = (x, y)
        accum_len = 0.0

        for px, py, pyaw in best_path:
            seg = euclid2((px, py), last_added)
            accum_len += seg
            if accum_len + 1e-6 >= spacing:
                p = Pose()
                p.position.x = px
                p.position.y = py
                p.position.z = 0.0
                p.orientation = yaw_to_quat(pyaw)
                poses.append(p)
                last_added = (px, py)
                accum_len = 0.0

        traj.poses = poses
        return traj


def main(args=None) -> None:
    """Entry point: initialize ROS 2, create node, and spin."""
    rclpy.init(args=args)
    node = TrajectoryCreation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
