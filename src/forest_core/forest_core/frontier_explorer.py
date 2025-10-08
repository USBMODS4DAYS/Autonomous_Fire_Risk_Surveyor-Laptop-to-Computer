#!/usr/bin/env python3
from typing import List, Tuple, Optional
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_cmd_vel', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('linear_speed_max', 1.0)
        self.declare_parameter('angular_speed_max', 1.2)
        self.declare_parameter('linear_k', 0.6)
        self.declare_parameter('angular_k', 1.2)
        self.declare_parameter('frontier_min_size', 15)
        self.declare_parameter('target_reached_dist', 0.6)
        qos_map = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_fast = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.map: Optional[OccupancyGrid] = None
        self.pose_xy: Tuple[float, float] = (0.0, 0.0)
        self.yaw: float = 0.0
        self.target_xy: Optional[Tuple[float, float]] = None
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.on_map, qos_map)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, qos_fast)
        if bool(self.get_parameter('use_cmd_vel').value):
            self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
            self.pub_goal = None
        else:
            self.pub_cmd = None
            self.pub_goal = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)
        self.timer = self.create_timer(0.5, self.tick)

    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose_xy = (x, y)
        self.yaw = yaw

    def tick(self):
        if self.map is None:
            return
        if self.target_xy is None or self.at_target():
            fronts = self.find_frontiers(self.map)
            if not fronts:
                if self.pub_cmd:
                    self.pub_cmd.publish(Twist())
                return
            self.target_xy = self.choose_target(fronts)
            if self.pub_goal:
                g = PoseStamped()
                g.header.frame_id = self.map.header.frame_id if self.map.header.frame_id else 'map'
                g.header.stamp = self.get_clock().now().to_msg()
                g.pose.position.x = self.target_xy[0]
                g.pose.position.y = self.target_xy[1]
                g.pose.orientation.w = 1.0
                self.pub_goal.publish(g)
        if self.pub_cmd and self.target_xy is not None:
            self.drive_toward(self.target_xy)

    def at_target(self) -> bool:
        tx, ty = self.target_xy if self.target_xy else (0.0, 0.0)
        x, y = self.pose_xy
        d = math.hypot(tx - x, ty - y)
        return d < float(self.get_parameter('target_reached_dist').value)

    def drive_toward(self, target_xy: Tuple[float, float]):
        x, y = self.pose_xy
        tx, ty = target_xy
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = self.angle_diff(desired_yaw, self.yaw)
        lin_k = float(self.get_parameter('linear_k').value)
        ang_k = float(self.get_parameter('angular_k').value)
        lin_max = float(self.get_parameter('linear_speed_max').value)
        ang_max = float(self.get_parameter('angular_speed_max').value)
        lin = max(0.0, min(lin_k * dist, lin_max))
        if abs(yaw_err) > math.radians(25.0):
            lin *= 0.2
        ang = max(-ang_max, min(ang_k * yaw_err, ang_max))
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.pub_cmd.publish(cmd)

    def angle_diff(self, a: float, b: float) -> float:
        d = (a - b + math.pi) % (2 * math.pi) - math.pi
        return d

    def find_frontiers(self, grid: OccupancyGrid) -> List[Tuple[float, float]]:
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = grid.data
        free = 0
        unknown = -1
        def idx(i, j):
            return j * w + i
        def is_valid(i, j):
            return 0 <= i < w and 0 <= j < h
        def is_frontier(i, j):
            if data[idx(i, j)] != free:
                return False
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni = i + di
                    nj = j + dj
                    if is_valid(ni, nj) and data[idx(ni, nj)] == unknown:
                        return True
            return False
        visited = [[False] * h for _ in range(w)]
        clusters: List[List[Tuple[int, int]]] = []
        for j in range(h):
            for i in range(w):
                if visited[i][j]:
                    continue
                if not is_frontier(i, j):
                    continue
                stack = [(i, j)]
                visited[i][j] = True
                cluster: List[Tuple[int, int]] = []
                while stack:
                    ci, cj = stack.pop()
                    cluster.append((ci, cj))
                    for di in (-1, 0, 1):
                        for dj in (-1, 0, 1):
                            if di == 0 and dj == 0:
                                continue
                            ni = ci + di
                            nj = cj + dj
                            if not is_valid(ni, nj):
                                continue
                            if visited[ni][nj]:
                                continue
                            if is_frontier(ni, nj):
                                visited[ni][nj] = True
                                stack.append((ni, nj))
                if len(cluster) >= int(self.get_parameter('frontier_min_size').value):
                    clusters.append(cluster)
        goals: List[Tuple[float, float]] = []
        for c in clusters:
            sx = 0.0
            sy = 0.0
            for (i, j) in c:
                wx = ox + (i + 0.5) * res
                wy = oy + (j + 0.5) * res
                sx += wx
                sy += wy
            cx = sx / len(c)
            cy = sy / len(c)
            goals.append((cx, cy))
        return goals

    def choose_target(self, goals: List[Tuple[float, float]]) -> Tuple[float, float]:
        x, y = self.pose_xy
        best = None
        best_d = float('inf')
        for gx, gy in goals:
            d = math.hypot(gx - x, gy - y)
            if d < best_d:
                best_d = d
                best = (gx, gy)
        return best if best is not None else (x, y)

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
