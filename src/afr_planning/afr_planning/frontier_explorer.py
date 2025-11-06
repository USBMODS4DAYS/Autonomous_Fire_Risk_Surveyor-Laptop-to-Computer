#!/usr/bin/env python3
"""
Frontier exploration node for the drone.

Subscribes:
  - map_topic (default: /drone/map)
  - TF: map_frame -> robot_frame

Publishes:
  - goal_topic : geometry_msgs/PoseStamped  (goal for A*)
  - /frontier_markers : visualization_msgs/MarkerArray (blue spheres for all clusters)
  - /frontier_target  : visualization_msgs/Marker (yellow sphere for chosen frontier)
"""

import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException

GridIndex = Tuple[int, int]


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        # === Parameters ===
        self.declare_parameter("map_topic", "/drone/map")
        self.declare_parameter("goal_topic", "/drone/goal")
        self.declare_parameter("map_frame", "drone/map")
        self.declare_parameter("robot_frame", "drone_base_link")

        self.declare_parameter("goal_altitude", 3.0)
        self.declare_parameter("min_frontier_size", 15)

        # 🔧 Increased tolerance (was 0.5 → now 1.0)
        self.declare_parameter("goal_reached_dist", 1.0)

        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("min_frontier_distance", 1.0)
        self.declare_parameter("frontier_clearance_cells", 2)
        self.declare_parameter("occ_threshold", 50)
        self.declare_parameter("unknown_gain_radius_cells", 6)
        self.declare_parameter("min_unknown_gain", 40)
        self.declare_parameter("known_path_penalty_cells", 5.0)

        map_topic = self.get_parameter("map_topic").value
        goal_topic = self.get_parameter("goal_topic").value

        # === Publishers / Subscribers ===
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, "frontier_markers", 10)
        self.frontier_target_pub = self.create_publisher(Marker, "frontier_target", 10)

        # === TF Buffer ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Internal state ===
        self.latest_map: Optional[OccupancyGrid] = None
        self.current_goal: Optional[PoseStamped] = None
        self.state: str = "IDLE"
        self.last_goal_send_time: Optional[Time] = None
        self.max_goal_time = Duration(seconds=60.0)
        self.ever_sent_goal = False

        timer_period = self.get_parameter("timer_period").value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"FrontierExplorer started. Subscribing to '{map_topic}', publishing goals to '{goal_topic}'."
        )

    # === Core callbacks ===
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def timer_callback(self) -> None:
        if self.latest_map is None:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        if self.state == "DONE":
            return

        if self.state == "IDLE":
            self.choose_and_send_new_frontier(robot_pose)
        elif self.state == "MOVING":
            if self.reached_goal(robot_pose):
                self.get_logger().info("Reached frontier goal. Selecting next one.")
                self.state = "IDLE"
                self.current_goal = None
            elif self.goal_timed_out():
                self.get_logger().warn("Goal timed out. Replanning.")
                self.state = "IDLE"
                self.current_goal = None

    # === TF & robot pose helpers ===
    def get_robot_pose(self) -> Optional[PoseStamped]:
        map_frame = self.get_parameter("map_frame").value
        robot_frame = self.get_parameter("robot_frame").value
        try:
            trans = self.tf_buffer.lookup_transform(
                map_frame, robot_frame, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header = trans.header
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation
        return pose

    def reached_goal(self, robot_pose: PoseStamped) -> bool:
        if self.current_goal is None:
            return False
        tol = self.get_parameter("goal_reached_dist").value
        dx = robot_pose.pose.position.x - self.current_goal.pose.position.x
        dy = robot_pose.pose.position.y - self.current_goal.pose.position.y
        dist = math.hypot(dx, dy)
        self.get_logger().info(f"Frontier reached check: dist={dist:.2f}, tol={tol:.2f}")
        return dist < tol

    def goal_timed_out(self) -> bool:
        if self.last_goal_send_time is None:
            return False
        return (self.get_clock().now() - self.last_goal_send_time) > self.max_goal_time

    # === Frontier selection ===
    def choose_and_send_new_frontier(self, robot_pose: PoseStamped) -> None:
        occ_grid = self.latest_map
        width, height = occ_grid.info.width, occ_grid.info.height
        data = list(occ_grid.data)
        res = occ_grid.info.resolution

        min_frontier_dist_m = self.get_parameter("min_frontier_distance").value
        min_frontier_dist_cells = min_frontier_dist_m / res
        clearance_cells = self.get_parameter("frontier_clearance_cells").value
        occ_threshold = self.get_parameter("occ_threshold").value
        gain_radius = self.get_parameter("unknown_gain_radius_cells").value
        min_unknown_gain = self.get_parameter("min_unknown_gain").value
        penalty_weight = self.get_parameter("known_path_penalty_cells").value

        # For the very first goal, allow close frontiers so we can get moving
        if not self.ever_sent_goal:
            min_frontier_dist_cells = 0.0

        # --- find frontier cells ---
        frontier_cells = find_frontier_cells(data, width, height)
        if not frontier_cells:
            self.get_logger().info("No frontiers found. Exploration complete.")
            self.state = "DONE"
            self.clear_markers()
            return

        clusters = cluster_frontiers(
            frontier_cells, width, height,
            self.get_parameter("min_frontier_size").value
        )
        if not clusters:
            self.get_logger().info("Frontiers found but all below min size.")
            self.state = "DONE"
            self.clear_markers()
            return

        robot_gx, robot_gy = world_to_grid(
            occ_grid.info,
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
        )

        def cluster_min_distance(cluster):
            return min(distance_grid(robot_gx, robot_gy, x, y) for (x, y) in cluster)

        # --- filter clusters ---
        valid_clusters = []
        for c in clusters:
            # distance filter
            if cluster_min_distance(c) < min_frontier_dist_cells:
                continue

            # unknown-gain filter: require enough unknown cells behind this frontier
            cx, cy = cluster_centroid(c)
            gain = count_unknown_in_radius(
                data, width, height, cx, cy, gain_radius
            )
            if gain < min_unknown_gain:
                continue

            # safe frontier cells (not hugging obstacles)
            safe_cells = [
                p for p in c if is_safe_frontier_cell(
                    data, width, height, p[0], p[1],
                    occ_threshold, clearance_cells
                )
            ]
            if safe_cells:
                valid_clusters.append((c, safe_cells))

        if not valid_clusters:
            self.get_logger().info("No valid frontier clusters (too close/unsafe/low gain).")
            self.state = "DONE"
            self.clear_markers()
            return

        # --- choose the best valid cluster: close + path that goes through unknown if possible ---
        def cluster_score(cluster):
            # distance in cells
            dist = cluster_min_distance(cluster)

            # unknown fraction along straight line to cluster centroid
            cx, cy = cluster_centroid(cluster)
            frac_unknown = unknown_fraction_along_line(
                data, width, height,
                robot_gx, robot_gy,
                cx, cy,
            )

            # Penalty for known: if all known -> (1 - frac_unknown) = 1
            penalty = penalty_weight * (1.0 - frac_unknown)

            return dist + penalty

        best_cluster, safe_cells = min(
            valid_clusters,
            key=lambda cs: cluster_score(cs[0])
        )

        # goal cell = safest cell in that best cluster, closest to robot
        target_gx, target_gy = min(
            safe_cells, key=lambda p: distance_grid(robot_gx, robot_gy, p[0], p[1])
        )

        # --- markers ---
        self.publish_frontier_markers(
            [c for (c, _) in valid_clusters],
            best_cluster,
            (target_gx, target_gy),
            occ_grid.info,
        )

        # --- send goal ---
        target_wx, target_wy = grid_to_world(occ_grid.info, target_gx, target_gy)
        goal_alt = self.get_parameter("goal_altitude").value

        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter("map_frame").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target_wx
        goal.pose.position.y = target_wy
        goal.pose.position.z = goal_alt
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.current_goal = goal
        self.state = "MOVING"
        self.last_goal_send_time = self.get_clock().now()
        self.ever_sent_goal = True

        self.get_logger().info(
            f"Published new frontier goal at ({target_wx:.2f}, {target_wy:.2f}) from cell ({target_gx},{target_gy})"
        )

    # === Marker helpers ===
    def clear_markers(self):
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.frontier_markers_pub.publish(ma)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.frontier_target_pub.publish(clear)

    def publish_frontier_markers(self, clusters, best_cluster, target_cell, map_info):
        map_frame = self.get_parameter("map_frame").value
        ma = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        # blue cluster centroids
        for i, cluster in enumerate(clusters):
            gx, gy = cluster_centroid(cluster)
            wx, wy = grid_to_world(map_info, gx, gy)
            m = Marker()
            m.header.frame_id = map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 0.0, 1.0, 0.8)
            ma.markers.append(m)

        self.frontier_markers_pub.publish(ma)

        # yellow goal
        tgx, tgy = target_cell
        twx, twy = grid_to_world(map_info, tgx, tgy)
        target = Marker()
        target.header.frame_id = map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.ns = "frontier_target"
        target.id = 0
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = twx
        target.pose.position.y = twy
        target.pose.position.z = 0.6
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.5
        target.color.r, target.color.g, target.color.b, target.color.a = (1.0, 1.0, 0.0, 0.9)
        self.frontier_target_pub.publish(target)


# === Frontier helper functions ===
def cell_index(x, y, width): return y * width + x
def get_cell(data, width, x, y): return data[cell_index(x, y, width)]
def distance_grid(x1, y1, x2, y2): return math.hypot(x2 - x1, y2 - y1)


def neighbors8(x, y, width, height):
    n = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                n.append((nx, ny))
    return n


def is_safe_frontier_cell(data, width, height, x, y, occ_threshold, clearance_cells):
    """Check if a frontier cell is not too close to obstacles."""
    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                if get_cell(data, width, nx, ny) >= occ_threshold:
                    return False
    return True


def find_frontier_cells(data, width, height):
    frontiers = []
    for y in range(height):
        for x in range(width):
            val = get_cell(data, width, x, y)
            if val != 0:  # must be free
                continue
            # at least one unknown neighbour
            if any(get_cell(data, width, nx, ny) == -1 for nx, ny in neighbors8(x, y, width, height)):
                frontiers.append((x, y))
    return frontiers


def cluster_frontiers(frontier_cells, width, height, min_size):
    s = set(frontier_cells)
    clusters = []
    while s:
        start = s.pop()
        q = [start]
        cluster = [start]
        while q:
            cx, cy = q.pop()
            for nx, ny in neighbors8(cx, cy, width, height):
                if (nx, ny) in s:
                    s.remove((nx, ny))
                    q.append((nx, ny))
                    cluster.append((nx, ny))
        if len(cluster) >= min_size:
            clusters.append(cluster)
    return clusters


def cluster_centroid(cluster):
    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    return int(round(sum(xs) / len(xs))), int(round(sum(ys) / len(ys)))


def count_unknown_in_radius(data, width, height, cx, cy, radius_cells):
    """Count unknown cells around (cx, cy) within a given radius in cells."""
    r2 = radius_cells * radius_cells
    count = 0
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > r2:
                continue
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < width and 0 <= ny < height:
                if get_cell(data, width, nx, ny) == -1:
                    count += 1
    return count


def unknown_fraction_along_line(data, width, height, x0, y0, x1, y1):
    """
    Approximate fraction of unknown cells along the straight line
    from (x0,y0) to (x1,y1) using a simple Bresenham walk.
    """
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1

    unknown = 0
    total = 0

    if dx >= dy:
        err = dx // 2
        while x != x1:
            if 0 <= x < width and 0 <= y < height:
                val = get_cell(data, width, x, y)
                if val == -1:
                    unknown += 1
                total += 1
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != y1:
            if 0 <= x < width and 0 <= y < height:
                val = get_cell(data, width, x, y)
                if val == -1:
                    unknown += 1
                total += 1
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    if total == 0:
        return 0.0
    return float(unknown) / float(total)


def grid_to_world(info, gx, gy):
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res


def world_to_grid(info, wx, wy):
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    return int((wx - ox) / res), int((wy - oy) / res)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
