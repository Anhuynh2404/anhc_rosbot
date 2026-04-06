"""
ANHC Planner Node v3 — waits for valid map before planning
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg            import OccupancyGrid, Path, Odometry
from geometry_msgs.msg       import PoseStamped
from visualization_msgs.msg  import Marker

from .anhc_astar import AStarPlanner


class AnhcPlannerNode(Node):

    def __init__(self):
        super().__init__('anhc_planner_node')

        self.declare_parameter('inflation_radius', 3)
        self.declare_parameter('heuristic', 'diagonal')
        self.declare_parameter('min_free_cells', 50)  # wait for this many free cells

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.heuristic        = self.get_parameter('heuristic').value
        self.min_free_cells   = self.get_parameter('min_free_cells').value

        self.planner    = None
        self.goal_pose  = None
        self.robot_x    = -7.0
        self.robot_y    =  0.0
        self.resolution = 0.1
        self.origin_x   = -10.0
        self.origin_y   = -10.0
        self.map_rows   = 0
        self.map_cols   = 0
        self.map_has_data = False   # True when map has enough free cells

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            OccupancyGrid, '/anhc/map', self.map_callback, map_qos)
        self.create_subscription(
            PoseStamped, '/anhc/goal', self.goal_callback, 10)
        self.create_subscription(
            Odometry, '/anhc/odom', self.odom_callback, 10)

        self.path_pub        = self.create_publisher(Path,   '/anhc/planned_path', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/anhc/goal_marker',  10)

        # Retry timer: re-plan every 3s if goal set but map not ready
        self.retry_timer = self.create_timer(3.0, self._retry_plan)

        self.get_logger().info('[Planner] Ready — waiting for map and goal')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.origin_x   = msg.info.origin.position.x
        self.origin_y   = msg.info.origin.position.y
        self.map_cols   = msg.info.width
        self.map_rows   = msg.info.height

        grid_2d = []
        for row_idx in range(self.map_rows):
            start = row_idx * self.map_cols
            row   = msg.data[start: start + self.map_cols]
            row   = [0 if v < 0 else int(v) for v in row]
            grid_2d.append(row)

        # Count free cells to know if map is useful
        free_count = sum(1 for row in grid_2d for v in row if v == 0)
        self.map_has_data = free_count >= self.min_free_cells

        if not self.map_has_data:
            self.get_logger().debug(
                f'[Planner] Map building... {free_count}/{self.min_free_cells} free cells')

        if self.planner is None:
            self.planner = AStarPlanner(
                grid_2d,
                resolution=self.resolution,
                inflation_radius=self.inflation_radius,
                heuristic=self.heuristic,
            )
        else:
            self.planner.update_grid(grid_2d)

        if self.goal_pose is not None and self.map_has_data:
            self._run_astar()

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(
            f'[Planner] Goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._publish_goal_marker(msg)

        if not self.map_has_data:
            self.get_logger().warn(
                '[Planner] Map not ready yet — will plan when map has enough data')
            return
        if self.planner is not None:
            self._run_astar()

    def _retry_plan(self):
        """Retry planning if goal is set but map wasn't ready before."""
        if (self.goal_pose is not None and
                self.planner is not None and
                self.map_has_data):
            # Check if path was already published successfully
            self._run_astar()

    def _run_astar(self):
        start = (self._w2r(self.robot_y), self._w2c(self.robot_x))
        goal  = (self._w2r(self.goal_pose.pose.position.y),
                 self._w2c(self.goal_pose.pose.position.x))

        if not self._valid(*start):
            self.get_logger().error(f'[Planner] Start {start} OOB')
            return
        if not self._valid(*goal):
            self.get_logger().error(f'[Planner] Goal {goal} OOB')
            return

        self.get_logger().info(
            f'[Planner] A* {start}→{goal} '
            f'(world ({self.robot_x:.1f},{self.robot_y:.1f})→'
            f'({self.goal_pose.pose.position.x:.1f},'
            f'{self.goal_pose.pose.position.y:.1f}))')

        try:
            cells = self.planner.plan(start, goal)
        except ValueError as e:
            self.get_logger().error(f'[Planner] {e}')
            return

        if cells is None:
            self.get_logger().warn('[Planner] ❌ No path found')
            return

        self.get_logger().info(
            f'[Planner] ✅ {len(cells)} cells '
            f'({len(cells)*self.resolution:.1f}m) | {self.planner.stats}')

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for (row, col) in cells:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x    = self._c2w(col)
            ps.pose.position.y    = self._r2w(row)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info('[Planner] Path → /anhc/planned_path')

    def _w2c(self, x):   return int((x - self.origin_x) / self.resolution)
    def _w2r(self, y):   return int((y - self.origin_y) / self.resolution)
    def _c2w(self, col): return self.origin_x + (col + 0.5) * self.resolution
    def _r2w(self, row): return self.origin_y + (row + 0.5) * self.resolution
    def _valid(self, r, c): return 0 <= r < self.map_rows and 0 <= c < self.map_cols

    def _publish_goal_marker(self, goal):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns, m.id        = 'anhc_goal', 0
        m.type            = Marker.CYLINDER
        m.action          = Marker.ADD
        m.pose            = goal.pose
        m.pose.position.z = 0.5
        m.scale.x = m.scale.y = 0.5
        m.scale.z = 1.0
        m.color.r = 1.0; m.color.g = 0.4; m.color.a = 0.85
        self.goal_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AnhcPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
