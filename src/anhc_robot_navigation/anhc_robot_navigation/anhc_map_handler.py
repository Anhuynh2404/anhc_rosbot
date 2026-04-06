"""
ANHC Map Handler v3
- Dùng tf2 để lấy sensor pose thay vì odom trực tiếp
- Fallback về odom nếu TF không available
- Handle LaserScan frame_id đúng cách
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg   import LaserScan
from nav_msgs.msg      import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose

try:
    from tf2_ros import Buffer, TransformListener
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class AnhcMapHandlerNode(Node):

    def __init__(self):
        super().__init__('anhc_map_handler_node')

        self.declare_parameter('map_width_m',    20.0)
        self.declare_parameter('map_height_m',   20.0)
        self.declare_parameter('map_resolution',  0.1)
        self.declare_parameter('map_origin_x',  -10.0)
        self.declare_parameter('map_origin_y',  -10.0)
        self.declare_parameter('publish_rate',    2.0)

        res = self.get_parameter('map_resolution').value
        w   = self.get_parameter('map_width_m').value
        h   = self.get_parameter('map_height_m').value
        self.origin_x   = self.get_parameter('map_origin_x').value
        self.origin_y   = self.get_parameter('map_origin_y').value
        self.resolution = res
        self.map_cols   = int(w / res)
        self.map_rows   = int(h / res)

        self.grid = np.full((self.map_rows, self.map_cols), -1, dtype=np.int8)

        # Robot pose from odom (fallback)
        self.robot_x   = -7.0
        self.robot_y   =  0.0
        self.robot_yaw =  0.0
        self.odom_received = False

        # TF2 for getting sensor pose
        if TF2_AVAILABLE:
            self.tf_buffer   = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(LaserScan, '/anhc/scan', self.scan_callback, 10)
        self.create_subscription(Odometry,  '/anhc/odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/anhc/map', map_qos)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_map)

        self.get_logger().info(
            f'[MapHandler] {self.map_rows}x{self.map_cols} @ {res}m/cell '
            f'origin=({self.origin_x},{self.origin_y})'
        )

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.odom_received = True

    def _get_sensor_pose(self, frame_id: str):
        """
        Get sensor pose in odom frame via TF2.
        Falls back to robot odom pose if TF not available.
        """
        if not TF2_AVAILABLE or not self.odom_received:
            return self.robot_x, self.robot_y, self.robot_yaw

        try:
            # Try to get transform from odom to sensor frame
            t = self.tf_buffer.lookup_transform(
                'odom', frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )
            return x, y, yaw
        except Exception:
            # Fallback to odom
            return self.robot_x, self.robot_y, self.robot_yaw

    def scan_callback(self, msg: LaserScan):
        if not self.odom_received:
            return

        # Get actual sensor position
        sx, sy, syaw = self._get_sensor_pose(msg.header.frame_id)

        rc = self._w2c(sx)
        rr = self._w2r(sy)
        angle = msg.angle_min

        for r in msg.ranges:
            angle += msg.angle_increment
            if math.isnan(r) or math.isinf(r):
                continue
            if not (msg.range_min <= r <= msg.range_max):
                continue

            world_angle = syaw + angle
            hx = sx + r * math.cos(world_angle)
            hy = sy + r * math.sin(world_angle)
            hc = self._w2c(hx)
            hr = self._w2r(hy)

            for fr, fc in self._bresenham(rr, rc, hr, hc)[:-1]:
                if self._ok(fr, fc) and self.grid[fr, fc] != 100:
                    self.grid[fr, fc] = 0

            if self._ok(hr, hc):
                self.grid[hr, hc] = 100

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width      = self.map_cols
        msg.info.height     = self.map_rows
        msg.info.origin     = Pose()
        msg.info.origin.position.x    = self.origin_x
        msg.info.origin.position.y    = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def _w2c(self, x): return int((x - self.origin_x) / self.resolution)
    def _w2r(self, y): return int((y - self.origin_y) / self.resolution)
    def _ok(self, r, c): return 0 <= r < self.map_rows and 0 <= c < self.map_cols

    @staticmethod
    def _bresenham(r0, c0, r1, c1):
        cells, dr, dc = [], abs(r1-r0), abs(c1-c0)
        r, c = r0, c0
        sr, sc = (1 if r1>r0 else -1), (1 if c1>c0 else -1)
        if dc > dr:
            err = dc // 2
            while c != c1:
                cells.append((r, c))
                err -= dr
                if err < 0: r += sr; err += dc
                c += sc
        else:
            err = dr // 2
            while r != r1:
                cells.append((r, c))
                err -= dc
                if err < 0: c += sc; err += dr
                r += sr
        cells.append((r1, c1))
        return cells


def main(args=None):
    rclpy.init(args=args)
    node = AnhcMapHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
