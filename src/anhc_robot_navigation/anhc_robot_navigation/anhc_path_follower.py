"""
╔══════════════════════════════════════════════════════════════════╗
║                 ANHC PATH FOLLOWER NODE                          ║
║                                                                  ║
║  Algorithm : Pure Pursuit with adaptive lookahead                ║
║                                                                  ║
║  Pure Pursuit principle:                                         ║
║    1. Find a "lookahead point" on the path ahead of the robot   ║
║    2. Compute the arc curvature κ needed to reach it            ║
║    3. Convert κ → angular velocity                               ║
║                                                                  ║
║  State machine:                                                  ║
║    IDLE → FOLLOWING → REACHED_GOAL                               ║
║              ↑              ↓                                    ║
║           new path        IDLE                                   ║
║                                                                  ║
║  Subscribes:                                                     ║
║    /anhc/planned_path  (nav_msgs/Path)                           ║
║    /anhc/odom          (nav_msgs/Odometry)                       ║
║  Publishes:                                                      ║
║    /anhc/cmd_vel       (geometry_msgs/Twist)                     ║
║    /anhc/follower/lookahead_marker  (visualization_msgs/Marker)  ║
╚══════════════════════════════════════════════════════════════════╝
"""

import math
import enum
from typing import Optional, List, Tuple

import rclpy
from rclpy.node   import Node
from rclpy.qos    import QoSProfile, ReliabilityPolicy

from nav_msgs.msg      import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker


# ════════════════════════════════════════════════════════════════
# STATE MACHINE
# ════════════════════════════════════════════════════════════════
class FollowerState(enum.Enum):
    IDLE          = 'IDLE'           # No path — robot stopped
    FOLLOWING     = 'FOLLOWING'      # Actively tracking path
    REACHED_GOAL  = 'REACHED_GOAL'  # Within goal_tolerance of final waypoint


# ════════════════════════════════════════════════════════════════
# PURE PURSUIT MATH (standalone, no ROS)
# ════════════════════════════════════════════════════════════════
class PurePursuit:
    """
    Pure Pursuit geometric controller.

    Given:
      - robot pose (x, y, yaw)
      - a list of waypoints
      - lookahead distance L

    Computes:
      - (linear_vel, angular_vel) to steer toward the lookahead point

    Math:
      Transform lookahead point into robot frame:
        lx =  cos(yaw)*(gx-rx) + sin(yaw)*(gy-ry)
        ly = -sin(yaw)*(gx-rx) + cos(yaw)*(gy-ry)

      Curvature:
        κ = 2 * ly / L²

      Angular velocity:
        ω = v * κ
    """

    @staticmethod
    def find_lookahead_point(
        robot_x:   float,
        robot_y:   float,
        waypoints: List[Tuple[float, float]],
        lookahead: float,
        last_idx:  int = 0,
    ) -> Tuple[Optional[Tuple[float, float]], int]:
        """
        Find the first waypoint at approximately `lookahead` distance
        ahead of the robot, starting from last_idx.

        Strategy:
          - Walk waypoints forward from last_idx
          - Return the first point whose distance ≥ lookahead
          - If no such point exists, return the last waypoint

        Returns:
            (lookahead_point, updated_last_idx)
            lookahead_point = None if waypoints is empty
        """
        if not waypoints:
            return None, last_idx

        # Advance last_idx to closest waypoint first
        # (avoids "catching up" to already-passed waypoints)
        min_dist = float('inf')
        closest  = last_idx
        for i in range(last_idx, len(waypoints)):
            wx, wy = waypoints[i]
            d = math.hypot(wx - robot_x, wy - robot_y)
            if d < min_dist:
                min_dist = d
                closest  = i

        # From closest point, find first that is ≥ lookahead distance
        for i in range(closest, len(waypoints)):
            wx, wy = waypoints[i]
            d = math.hypot(wx - robot_x, wy - robot_y)
            if d >= lookahead:
                return (wx, wy), i

        # No waypoint far enough — return last waypoint
        return waypoints[-1], len(waypoints) - 1

    @staticmethod
    def compute_velocity(
        robot_x:        float,
        robot_y:        float,
        robot_yaw:      float,
        lookahead_x:    float,
        lookahead_y:    float,
        lookahead_dist: float,
        max_linear:     float,
        max_angular:    float,
    ) -> Tuple[float, float]:
        """
        Compute (linear_vel, angular_vel) using Pure Pursuit.

        Args:
            robot_x/y/yaw    : current robot pose
            lookahead_x/y    : target point in world frame
            lookahead_dist   : desired lookahead distance L
            max_linear       : max forward speed (m/s)
            max_angular      : max rotation speed (rad/s)

        Returns:
            (linear_vel, angular_vel)
        """
        # ── Transform lookahead into robot frame ─────────────────
        dx = lookahead_x - robot_x
        dy = lookahead_y - robot_y

        # Rotate world-frame offset into robot frame
        lx =  math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
        ly = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy

        # ── Actual distance to lookahead point ────────────────────
        L = math.hypot(lx, ly)
        if L < 1e-6:
            return 0.0, 0.0

        # ── Curvature κ = 2*ly / L² ───────────────────────────────
        # Positive ly → point is to the left → positive ω (turn left)
        # Negative ly → point is to the right → negative ω (turn right)
        curvature = 2.0 * ly / (L * L)

        # ── Adaptive linear speed ─────────────────────────────────
        # Slow down when turning sharply
        # |curvature| = 0 → straight → full speed
        # |curvature| large → sharp turn → slow down
        turn_factor  = 1.0 / (1.0 + 2.0 * abs(curvature))
        linear_vel   = max_linear * turn_factor
        linear_vel   = max(0.05, min(linear_vel, max_linear))

        # ── Angular velocity ──────────────────────────────────────
        angular_vel = linear_vel * curvature
        angular_vel = max(-max_angular, min(angular_vel, max_angular))

        # ── If robot is facing away from lookahead → rotate in place ─
        if lx < 0:
            linear_vel  = 0.0
            angular_vel = max_angular * (1.0 if ly > 0 else -1.0)

        return linear_vel, angular_vel


# ════════════════════════════════════════════════════════════════
# ROS 2 NODE
# ════════════════════════════════════════════════════════════════
class AnhcPathFollowerNode(Node):

    def __init__(self):
        super().__init__('anhc_path_follower_node')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('max_linear_vel',      0.4)   # m/s
        self.declare_parameter('max_angular_vel',     1.2)   # rad/s
        self.declare_parameter('goal_tolerance',      0.25)  # m
        self.declare_parameter('waypoint_tolerance',  0.25)  # m
        self.declare_parameter('lookahead_distance',  0.6)   # m
        self.declare_parameter('min_lookahead',       0.3)   # m
        self.declare_parameter('max_lookahead',       1.5)   # m
        self.declare_parameter('control_frequency',  20.0)   # Hz
        self.declare_parameter('stuck_timeout',       5.0)   # s
        self.declare_parameter('stuck_dist_thresh',   0.05)  # m

        self.max_linear    = self.get_parameter('max_linear_vel').value
        self.max_angular   = self.get_parameter('max_angular_vel').value
        self.goal_tol      = self.get_parameter('goal_tolerance').value
        self.wp_tol        = self.get_parameter('waypoint_tolerance').value
        self.lookahead     = self.get_parameter('lookahead_distance').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.ctrl_freq     = self.get_parameter('control_frequency').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_thresh  = self.get_parameter('stuck_dist_thresh').value

        # ── State ────────────────────────────────────────────────
        self.state:      FollowerState          = FollowerState.IDLE
        self.waypoints:  List[Tuple[float,float]] = []
        self.last_idx:   int                    = 0
        self.robot_x:    float                  = -7.0
        self.robot_y:    float                  = 0.0
        self.robot_yaw:  float                  = 0.0

        # Anti-stuck detection
        self.last_progress_x:    float = -7.0
        self.last_progress_y:    float =  0.0
        self.last_progress_time: float =  0.0

        # ── Subscribers ───────────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, '/anhc/planned_path',
            self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/anhc/odom',
            self.odom_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/anhc/cmd_vel', 10
        )
        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/anhc/follower/lookahead_marker', 10
        )

        # ── Control loop timer ────────────────────────────────────
        self.control_timer = self.create_timer(
            1.0 / self.ctrl_freq,
            self.control_loop
        )

        self.get_logger().info(
            f'[PathFollower] Ready | '
            f'v_max={self.max_linear}m/s | '
            f'ω_max={self.max_angular}rad/s | '
            f'lookahead={self.lookahead}m'
        )

    # ── Odometry callback ─────────────────────────────────────────
    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # ── Path callback ─────────────────────────────────────────────
    def path_callback(self, msg: Path) -> None:
        """
        Receive new path from A* planner.
        Extract waypoints and transition to FOLLOWING state.
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('[PathFollower] Received empty path')
            return

        # Extract (x, y) waypoints
        self.waypoints = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]
        self.last_idx  = 0
        self.state     = FollowerState.FOLLOWING

        # Reset stuck detection
        self.last_progress_x    = self.robot_x
        self.last_progress_y    = self.robot_y
        self.last_progress_time = self._now()

        self.get_logger().info(
            f'[PathFollower] New path received: {len(self.waypoints)} waypoints | '
            f'Start: ({self.waypoints[0][0]:.2f},{self.waypoints[0][1]:.2f}) → '
            f'Goal:  ({self.waypoints[-1][0]:.2f},{self.waypoints[-1][1]:.2f})'
        )

    # ════════════════════════════════════════════════════════════
    # MAIN CONTROL LOOP  (runs at ctrl_freq Hz)
    # ════════════════════════════════════════════════════════════
    def control_loop(self) -> None:
        """
        State machine dispatcher.
        Called at control_frequency Hz by the timer.
        """
        if self.state == FollowerState.IDLE:
            self._stop_robot()

        elif self.state == FollowerState.FOLLOWING:
            self._follow_path()

        elif self.state == FollowerState.REACHED_GOAL:
            self._stop_robot()
            self.state = FollowerState.IDLE
            self.get_logger().info(
                '[PathFollower] 🏁 Goal reached! Robot stopped.'
            )

    # ── FOLLOWING state ───────────────────────────────────────────
    def _follow_path(self) -> None:
        """
        Execute one Pure Pursuit control step.

        Steps:
          1. Check if final goal reached → transition to REACHED_GOAL
          2. Compute adaptive lookahead distance
          3. Find lookahead point on path
          4. Compute and publish cmd_vel
          5. Check stuck condition
        """
        if not self.waypoints:
            self.state = FollowerState.IDLE
            return

        goal_x, goal_y = self.waypoints[-1]

        # ── 1. Check if goal reached ─────────────────────────────
        dist_to_goal = math.hypot(
            goal_x - self.robot_x,
            goal_y - self.robot_y
        )
        if dist_to_goal <= self.goal_tol:
            self.state = FollowerState.REACHED_GOAL
            self._stop_robot()
            return

        # ── 2. Adaptive lookahead ─────────────────────────────────
        # Increase lookahead at high speed, decrease when nearly stopped
        # This helps smooth the path at straight sections and
        # be more precise near the goal
        speed_ratio  = min(1.0, dist_to_goal / 2.0)
        lookahead    = self.min_lookahead + speed_ratio * (
            self.max_lookahead - self.min_lookahead
        )

        # ── 3. Find lookahead point ───────────────────────────────
        lh_point, self.last_idx = PurePursuit.find_lookahead_point(
            self.robot_x, self.robot_y,
            self.waypoints,
            lookahead,
            self.last_idx,
        )

        if lh_point is None:
            self._stop_robot()
            return

        lh_x, lh_y = lh_point

        # ── 4. Compute velocities ─────────────────────────────────
        linear_vel, angular_vel = PurePursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw,
            lh_x, lh_y, lookahead,
            self.max_linear, self.max_angular,
        )

        self._publish_cmd_vel(linear_vel, angular_vel)
        self._publish_lookahead_marker(lh_x, lh_y)

        # ── 5. Stuck detection ────────────────────────────────────
        progress = math.hypot(
            self.robot_x - self.last_progress_x,
            self.robot_y - self.last_progress_y,
        )
        now = self._now()

        if progress > self.stuck_thresh:
            # Made progress → reset timer
            self.last_progress_x    = self.robot_x
            self.last_progress_y    = self.robot_y
            self.last_progress_time = now
        else:
            elapsed = now - self.last_progress_time
            if elapsed > self.stuck_timeout:
                self.get_logger().warn(
                    f'[PathFollower] ⚠️  Robot appears stuck for '
                    f'{elapsed:.1f}s — stopping. Re-send goal to retry.'
                )
                self._stop_robot()
                self.state    = FollowerState.IDLE
                self.waypoints = []

        # ── Debug logging (every ~1s) ─────────────────────────────
        if int(now * self.ctrl_freq) % int(self.ctrl_freq) == 0:
            self.get_logger().info(
                f'[PathFollower] '
                f'pos=({self.robot_x:.2f},{self.robot_y:.2f}) '
                f'yaw={math.degrees(self.robot_yaw):.1f}° | '
                f'lh=({lh_x:.2f},{lh_y:.2f}) '
                f'dist_goal={dist_to_goal:.2f}m | '
                f'v={linear_vel:.2f} ω={angular_vel:.2f}'
            )

    # ── Helper: publish cmd_vel ───────────────────────────────────
    def _publish_cmd_vel(self,
                         linear: float,
                         angular: float) -> None:
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    # ── Helper: stop robot ────────────────────────────────────────
    def _stop_robot(self) -> None:
        self._publish_cmd_vel(0.0, 0.0)

    # ── Helper: lookahead marker for RViz ─────────────────────────
    def _publish_lookahead_marker(self, x: float, y: float) -> None:
        marker = Marker()
        marker.header.frame_id    = 'odom'
        marker.header.stamp       = self.get_clock().now().to_msg()
        marker.ns                 = 'anhc_lookahead'
        marker.id                 = 1
        marker.type               = Marker.SPHERE
        marker.action             = Marker.ADD
        marker.pose.position.x    = x
        marker.pose.position.y    = y
        marker.pose.position.z    = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x            = 0.25
        marker.scale.y            = 0.25
        marker.scale.z            = 0.25
        marker.color.r            = 0.0
        marker.color.g            = 1.0
        marker.color.b            = 1.0
        marker.color.a            = 0.9
        self.lookahead_marker_pub.publish(marker)

    # ── Helper: current time in seconds ───────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = AnhcPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
