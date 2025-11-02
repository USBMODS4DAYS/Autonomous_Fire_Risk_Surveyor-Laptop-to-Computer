#!/usr/bin/env python3
import math, heapq
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
from collections import deque

def heuristic(a: Tuple[int,int], b: Tuple[int,int]) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def neighbors_8(i: int, j: int):
    rt2 = math.sqrt(2.0)
    return [(i-1,j-1,rt2),(i-1,j,1.0),(i-1,j+1,rt2),
            (i,  j-1,1.0),             (i,  j+1,1.0),
            (i+1,j-1,rt2),(i+1,j,1.0),(i+1,j+1,rt2)]

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # Frames / topics
        self.fixed_frame = 'drone/map'
        self.base_link   = 'drone_base_link'
        self.map_topic   = '/map'          # slam_toolbox publishes /map (frame_id = drone/map)
        self.goal_topic  = '/drone/goal'

        # Params
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('inflate_radius', 0.0)           # m
        self.declare_parameter('plan_rate_hz', 1.0)
        self.declare_parameter('unknown_policy', 'blocked')      # 'blocked' | 'free' | 'penalized'
        self.declare_parameter('unknown_penalty', 2.0)           # extra cost per unknown step (if penalized)
        self.declare_parameter('snap_goal_radius_m', 0.6)        # search radius to snap goal to nearest free
        self.declare_parameter('inflate_ignore_unknown', True)   # ignore unknown for inflation

        self.occ_threshold         = int(self.get_parameter('occ_threshold').value)
        self.inflate_radius        = float(self.get_parameter('inflate_radius').value)
        self.plan_period           = 1.0/float(self.get_parameter('plan_rate_hz').value)
        self.unknown_policy        = str(self.get_parameter('unknown_policy').value)
        self.unknown_penalty       = float(self.get_parameter('unknown_penalty').value)
        self.snap_goal_radius_m    = float(self.get_parameter('snap_goal_radius_m').value)
        self.inflate_ignore_unknown= bool(self.get_parameter('inflate_ignore_unknown').value)

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IO
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 5)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 5)
        self.path_pub        = self.create_publisher(Path,   '/drone/astar_path',    10)
        self.path_marker_pub = self.create_publisher(Marker, '/drone/astar_markers', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/drone/goal_marker',   10)

        # State
        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_xy_world: Optional[Tuple[float,float]] = None
        self.logged_map_header = False

        self.create_timer(self.plan_period, self._try_plan)
        self.get_logger().info("A* ready. Publish PoseStamped to /drone/goal with frame_id=drone/map.")

    # ---------- Callbacks ----------
    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        if not self.logged_map_header:
            mi = msg.info
            xmin, ymin = mi.origin.position.x, mi.origin.position.y
            xmax, ymax = xmin + mi.width*mi.resolution, ymin + mi.height*mi.resolution
            self.get_logger().info(f"/map frame='{msg.header.frame_id}' res={mi.resolution:.3f} "
                                   f"size={mi.width}x{mi.height} world_bounds=([{xmin:.2f},{xmax:.2f}], "
                                   f"[{ymin:.2f},{ymax:.2f}])")
            self.logged_map_header = True

    def _on_goal(self, msg: PoseStamped):
        if msg.header.frame_id != self.fixed_frame:
            self.get_logger().warn(f"Goal must be in {self.fixed_frame}, got {msg.header.frame_id}")
            return
        self.goal_xy_world = (msg.pose.position.x, msg.pose.position.y)
        self.goal_marker_pub.publish(self._make_goal_marker(self.goal_xy_world))
        self.get_logger().info(f"Received goal: {self.goal_xy_world}")

    # ---------- Planning ----------
    def _try_plan(self):
        if self.map_msg is None or self.goal_xy_world is None:
            return

        start_xy = self._get_start_xy_world()
        if start_xy is None:
            return
        sx, sy = start_xy
        gx, gy = self.goal_xy_world

        mi = self.map_msg.info
        start_ij = self.world_to_grid(sx, sy)
        goal_ij  = self.world_to_grid(gx, gy)

        if start_ij is None or goal_ij is None:
            self.get_logger().warn("Start or goal outside the known map extents.")
            return

        si, sj = start_ij
        gi, gj = goal_ij
        start_cell = self.map_msg.data[si*mi.width + sj]
        goal_cell  = self.map_msg.data[gi*mi.width + gj]
        self.get_logger().info(f"indices start=({si},{sj}) val={start_cell}  goal=({gi},{gj}) val={goal_cell}")

        free_mask, unknown_mask = self._build_masks()

        # Snap goal to nearest free if needed
        if not free_mask[gi][gj]:
            snapped = self._snap_goal_to_free((gi, gj), free_mask)
            if snapped:
                gi, gj = snapped
                gx, gy = self.grid_to_world(gi, gj)
                self.goal_xy_world = (gx, gy)
                self.goal_marker_pub.publish(self._make_goal_marker(self.goal_xy_world))
                self.get_logger().info(f"Goal snapped to free at indices=({gi},{gj}) world=({gx:.2f},{gy:.2f})")
            else:
                self.get_logger().warn("No nearby free cell for goal; planning aborted.")
                return

        path_cells = self._astar(free_mask, unknown_mask, start_ij, (gi, gj))
        if not path_cells:
            self.get_logger().warn("A* failed: no path.")
            return

        xy_world = [self.grid_to_world(i, j) for (i, j) in path_cells]
        self.path_pub.publish(self._make_path_msg(xy_world))
        self.path_marker_pub.publish(self._make_sphere_list_marker(xy_world))

    # ---------- TF ----------
    def _get_start_xy_world(self) -> Optional[Tuple[float,float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, self.base_link, rclpy.time.Time())
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn_throttle(2000, f"TF {self.fixed_frame}->{self.base_link} not ready: {e}")
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    # ---------- Map conversions ----------
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int,int]]:
        m = self.map_msg.info
        j = int((x - m.origin.position.x) / m.resolution)
        i = int((y - m.origin.position.y) / m.resolution)
        if 0 <= i < m.height and 0 <= j < m.width:
            return (i, j)
        return None

    def grid_to_world(self, i: int, j: int) -> Tuple[float,float]:
        m = self.map_msg.info
        x = m.origin.position.x + (j + 0.5) * m.resolution
        y = m.origin.position.y + (i + 0.5) * m.resolution
        return (x, y)

    # ---------- Masks & inflation ----------
    def _build_masks(self):
        """Returns (free_mask, unknown_mask). free_mask[i][j]=True means traversable."""
        data = self.map_msg.data
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        res = self.map_msg.info.resolution
        rad_cells = int(max(0.0, self.inflate_radius) / res + 0.5)

        free = [[True]*w for _ in range(h)]
        unknown = [[False]*w for _ in range(h)]

        for i in range(h):
            for j in range(w):
                v = data[i*w + j]
                if v < 0:
                    if self.unknown_policy == 'blocked':
                        free[i][j] = False
                    elif self.unknown_policy == 'free':
                        free[i][j] = True
                    else:  # penalized
                        free[i][j] = True
                        unknown[i][j] = True
                else:
                    free[i][j] = (v < self.occ_threshold)

        if rad_cells > 0:
            r2 = rad_cells * rad_cells
            for i in range(h):
                for j in range(w):
                    v = data[i*w + j]
                    is_occ = (v >= self.occ_threshold) if v >= 0 else (not self.inflate_ignore_unknown)
                    if not is_occ:
                        continue
                    for di in range(-rad_cells, rad_cells+1):
                        ii = i + di
                        if ii < 0 or ii >= h: 
                            continue
                        # circle: di^2 + dj^2 <= r^2
                        max_dj = int((r2 - di*di)**0.5)
                        for dj in range(-max_dj, max_dj+1):
                            jj = j + dj
                            if 0 <= jj < w:
                                free[ii][jj] = False
        return free, unknown

    # ---------- Goal snapping ----------
    def _snap_goal_to_free(self, goal: Tuple[int,int], free_mask) -> Optional[Tuple[int,int]]:
        gi, gj = goal
        if free_mask[gi][gj]:
            return goal
        m = self.map_msg.info
        max_cells = int(self.snap_goal_radius_m / m.resolution + 0.5)
        visited = set([(gi, gj)])
        q = deque([(gi, gj, 0)])
        while q:
            i, j, d = q.popleft()
            if d > max_cells: 
                break
            if 0 <= i < m.height and 0 <= j < m.width and free_mask[i][j]:
                return (i, j)
            for di, dj, _ in neighbors_8(i, j):
                ni, nj = di, dj  # careful: neighbors_8 already returns absolute indices when fed absolute…
        # Correct neighbor expansion:
        visited = set([(gi, gj)])
        q = deque([(gi, gj, 0)])
        while q:
            i, j, d = q.popleft()
            if d > max_cells:
                break
            if 0 <= i < m.height and 0 <= j < m.width and free_mask[i][j]:
                return (i, j)
            for oi, oj, _ in neighbors_8(0, 0):
                ni, nj = i + oi, j + oj
                if (ni, nj) not in visited:
                    visited.add((ni, nj))
                    q.append((ni, nj, d+1))
        return None

    # ---------- A* ----------
    def _astar(self, free_mask, unknown_mask, start, goal):
        h, w = len(free_mask), len(free_mask[0])
        si, sj = start
        gi, gj = goal
        if not (0 <= si < h and 0 <= sj < w and 0 <= gi < h and 0 <= gj < w):
            return []

        if not free_mask[si][sj] or not free_mask[gi][gj]:
            return []

        penalize = (self.unknown_policy == 'penalized')
        unk_pen  = self.unknown_penalty

        openq = []
        heapq.heappush(openq, (0.0, start))
        g_cost = {start: 0.0}
        parent = {start: None}
        closed = set()

        while openq:
            _, cur = heapq.heappop(openq)
            if cur in closed:
                continue
            closed.add(cur)

            if cur == goal:
                path = []
                c = cur
                while c is not None:
                    path.append(c)
                    c = parent[c]
                return list(reversed(path))

            ci, cj = cur
            for di, dj, step in neighbors_8(0, 0):
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < h and 0 <= nj < w):
                    continue
                if not free_mask[ni][nj]:
                    continue
                extra = (unk_pen if penalize and unknown_mask[ni][nj] else 0.0)
                tentative = g_cost[cur] + step + extra
                nxt = (ni, nj)
                if nxt not in g_cost or tentative < g_cost[nxt]:
                    g_cost[nxt] = tentative
                    parent[nxt] = cur
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(openq, (f, nxt))
        return []

    # ---------- Viz ----------
    def _make_path_msg(self, xy):
        msg = Path()
        msg.header.frame_id = self.fixed_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy:
            ps = PoseStamped()
            ps.header.frame_id = self.fixed_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _make_sphere_list_marker(self, xy, ns="astar_path", mid=0, scale=0.08, rgba=(0.1,0.6,1.0,1.0)):
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = ns, mid
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        for x, y in xy:
            p = Point(x=float(x), y=float(y), z=0.05)
            m.points.append(p)
        return m

    def _make_goal_marker(self, goal_xy, ns="astar_goal", mid=1, scale=0.25, rgba=(1.0,0.2,0.2,1.0)):
        x, y = goal_xy
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = ns, mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x); m.pose.position.y = float(y); m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
