import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

FREE, OCC, UNKNOWN = 0, 100, -1


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.control_step)  # 20 Hz

        self.map = None
        self.map_info = None
        self.goal = None
        self.path_pts = []

        # follower parameters
        self.v_max, self.w_max = 0.25, 0.6
        self.k_v, self.k_w = 0.6, 0.9
        self.lookahead = 0.4

        # stabilisation parameters
        self.STRAFE_ENABLED = True
        self.ex_f = 0.0
        self.ey_f = 0.0
        self.alpha = 0.5  # low-pass filter coefficient

        self.get_logger().info('A* planner ready — click a 2D Goal Pose in RViz.')

    # ---------- map + goal callbacks ----------
    def on_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'New goal: {self.goal}')
        self.plan_path()

    # ---------- path planning ----------
    def plan_path(self):
        if self.map is None or self.map_info is None:
            self.get_logger().warn('No map yet')
            return

        start = self.get_pose_in_map()
        if start is None:
            self.get_logger().warn('No TF map→base_link yet')
            return

        sx, sy, _ = start
        gx, gy = self.goal
        s_g = self.world_to_grid(sx, sy)
        g_g = self.world_to_grid(gx, gy)

        if not self.in_bounds(*s_g) or not self.in_bounds(*g_g):
            self.get_logger().warn('Start or goal out of bounds')
            return

        grid_free = self.inflate_obstacles(
            self.map, radius_cells=max(1, int(0.25 / self.map_info.resolution))
        )
        path_cells = self.astar(grid_free, s_g, g_g)
        if not path_cells:
            self.get_logger().warn('No path found')
            self.path_pts = []
            self.publish_path([])
            return

        pts = [self.grid_to_world(cx, cy) for (cy, cx) in path_cells]  # row-major
        self.path_pts = self.sparsify(pts, step=2)
        self.lookahead = max(0.4, 10 * self.map_info.resolution)
        self.publish_path(self.path_pts)

    def astar(self, grid, start, goal):
        H, W = grid.shape
        sy, sx = start[1], start[0]
        gy, gx = goal[1], goal[0]
        if grid[sy, sx] or grid[gy, gx]:
            return []

        from heapq import heappush, heappop

        def h(y, x):
            dy, dx = abs(y - gy), abs(x - gx)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

        neigh = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]
        cost = {(0, 1): 1, (1, 0): 1, (0, 0): 1, (1, 1): math.sqrt(2)}

        openset = []
        heappush(openset, (0.0, (sy, sx)))
        gscore = {(sy, sx): 0.0}
        parent = {(sy, sx): None}

        while openset:
            _, (cy, cx) = heappop(openset)
            if (cy, cx) == (gy, gx):
                path = []
                cur = (cy, cx)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                return list(reversed(path))

            for dy, dx in neigh:
                ny, nx = cy + dy, cx + dx
                if ny < 0 or nx < 0 or ny >= H or nx >= W:
                    continue
                if grid[ny, nx]:
                    continue
                step = cost[(abs(dx), abs(dy))]
                tentative = gscore[(cy, cx)] + step
                if (ny, nx) not in gscore or tentative < gscore[(ny, nx)]:
                    gscore[(ny, nx)] = tentative
                    parent[(ny, nx)] = (cy, cx)
                    heappush(openset, (tentative + h(ny, nx), (ny, nx)))
        return []

    # ---------- holonomic control ----------
    def control_step(self):
        # No path? stop.
        if not self.path_pts:
            self.cmd_pub.publish(Twist())
            return

        pose = self.get_pose_in_map()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return

        x, y, yaw = pose

        # ----- select target point -----
        final = self.path_pts[-1]
        dist_goal = self.dist((x, y), final)

        # scale numbers from map resolution
        cell = self.map_info.resolution
        capture_R  = max(0.40, 3 * cell)
        approach_R = max(0.80, 8 * cell)

        if dist_goal > approach_R:
            target = final
            for p in self.path_pts:
                if self.dist((x, y), p) > self.lookahead:
                    target = p
                    break
        else:
            target = final

        # ----- errors in body frame -----
        dx = target[0] - x
        dy = target[1] - y
        ex =  math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # low-pass filter to avoid twitch
        self.ex_f = self.alpha * ex + (1.0 - self.alpha) * self.ex_f
        self.ey_f = self.alpha * ey + (1.0 - self.alpha) * self.ey_f
        ex, ey = self.ex_f, self.ey_f

        heading_err = math.atan2(ey, ex)

        # deadbands
        dead_xy = max(0.08, 1.5 * cell)
        if abs(ex) < dead_xy:
            ex = 0.0
        if abs(ey) < dead_xy:
            ey = 0.0

        # distance-based taper (brake as we get close)
        taper_R = 1.2
        scale   = max(0.0, min(1.0, dist_goal / taper_R))

        cmd = Twist()

        if self.STRAFE_ENABLED:
            k_pos_far = 0.8
            k_pos_near = 0.5

            if dist_goal > approach_R:
                kpos = k_pos_far
                yaw_scale = max(0.25, scale)
                wz = 0.35 * self.k_w * heading_err * yaw_scale
            else:
                kpos = k_pos_near
                yaw_scale = max(0.2, scale)
                wz = 0.28 * self.k_w * heading_err * yaw_scale

            vx = kpos * ex * scale
            vy = kpos * ey * scale

            # clamps
            vx = max(-self.v_max, min(self.v_max, vx))
            vy = max(-self.v_max, min(self.v_max, vy))
            wz = max(-self.w_max, min(self.w_max, wz))

            # small command suppression
            if abs(vx) < 0.03:
                vx = 0.0
            if abs(vy) < 0.03:
                vy = 0.0
            if abs(wz) < 0.03:
                wz = 0.0

            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz
        else:
            theta_spin = 1.0
            if abs(heading_err) > theta_spin:
                v = 0.0
                w = max(-self.w_max, min(self.w_max, self.k_w * heading_err))
            else:
                v = max(0.0, min(self.v_max, self.k_v * ex * scale))
                w = max(-self.w_max, min(self.w_max, 0.6 * self.k_w * heading_err * max(0.2, scale)))
            if abs(v) < 0.03:
                v = 0.0
            if abs(w) < 0.03:
                w = 0.0
            cmd.linear.x = v
            cmd.angular.z = w

        # ----- capture -----
        if dist_goal < capture_R:
            self.cmd_pub.publish(Twist())
            self.path_pts = []
            return

        self.cmd_pub.publish(cmd)

    # ---------- utilities ----------
    def get_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def publish_path(self, pts):
        path = Path()
        path.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        for (wx, wy) in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def world_to_grid(self, wx, wy):
        r = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        return int((wx - ox) / r), int((wy - oy) / r)

    def grid_to_world(self, gx, gy):
        r = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        return ox + (gx + 0.5) * r, oy + (gy + 0.5) * r

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height

    def inflate_obstacles(self, raw_grid, radius_cells=3):
        occ = (raw_grid == OCC) | (raw_grid == UNKNOWN)
        H, W = occ.shape
        dil = occ.copy()
        r = radius_cells
        if r <= 0:
            return dil.astype(np.uint8)

        pad = np.pad(occ, r, mode='edge')
        for y in range(H):
            for x in range(W):
                window = pad[y:y + 2 * r + 1, x:x + 2 * r + 1]
                if window.any():
                    dil[y, x] = True
        return dil.astype(np.uint8)

    def sparsify(self, pts, step=2):
        if not pts:
            return pts
        return [p for i, p in enumerate(pts) if i % step == 0] + [pts[-1]]

    @staticmethod
    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
