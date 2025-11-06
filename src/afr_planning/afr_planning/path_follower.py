#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ExtrapolationException,
    ConnectivityException,
)


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")

        # Frames
        self.fixed_frame = "drone/map"
        self.base_link = "drone_base_link"

        # Parameters
        self.declare_parameter("max_lin_speed", 0.4)
        self.declare_parameter("max_ang_speed", 0.8)
        self.declare_parameter("look_ahead_dist", 0.6)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("control_rate_hz", 20.0)

        self.max_lin_speed = float(self.get_parameter("max_lin_speed").value)
        self.max_ang_speed = float(self.get_parameter("max_ang_speed").value)
        self.look_ahead_dist = float(self.get_parameter("look_ahead_dist").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.control_period = 1.0 / float(
            self.get_parameter("control_rate_hz").value
        )

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IO
        self.cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.path_sub = self.create_subscription(
            Path, "/drone/astar_path", self._on_path, 10
        )

        # State
        self.path: List[Tuple[float, float]] = []
        self.goal_xy: Optional[Tuple[float, float]] = None

        # Control loop timer
        self.timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            "Path follower ready. It will follow /drone/astar_path using /drone/cmd_vel."
        )

    # -------- Path callback --------
    def _on_path(self, msg: Path):
        pts: List[Tuple[float, float]] = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self.path = pts
        self.goal_xy = pts[-1] if pts else None

        self.get_logger().info(
            f"New path received: {len(self.path)} points. Goal = {self.goal_xy}"
        )

    # -------- TF helper --------
    def _get_pose_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_link, rclpy.time.Time()
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn(f"TF {self.fixed_frame}->{self.base_link} not ready: {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (t.x, t.y, yaw)

    # -------- Control loop --------
    def _control_loop(self):
        # No path: stop
        if not self.path or self.goal_xy is None:
            self._publish_cmd(0.0, 0.0)
            return

        pose = self._get_pose_map()
        if pose is None:
            self._publish_cmd(0.0, 0.0)
            return
        x, y, yaw = pose

        # Check goal reached
        gx, gy = self.goal_xy
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.goal_tolerance:
            self._publish_cmd(0.0, 0.0)
            return

        # Find lookahead target on path
        target = self._find_lookahead(x, y)
        if target is None:
            self._publish_cmd(0.0, 0.0)
            return
        tx, ty = target

        # Vector from robot to target in map frame
        dx = tx - x
        dy = ty - y

        # Transform into base_link frame (rotate by -yaw)
        dx_b = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        dy_b = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        angle_to_target = math.atan2(dy_b, dx_b)

        # Forward speed proportional to how "in front" the target is
        v = self.max_lin_speed * math.cos(angle_to_target)
        if v < 0.0:
            v = 0.0  # don't drive backwards

        # Turn towards target
        omega = 2.0 * angle_to_target  # simple gain

        # Clamp speeds
        v = max(-self.max_lin_speed, min(self.max_lin_speed, v))
        omega = max(-self.max_ang_speed, min(self.max_ang_speed, omega))

        self._publish_cmd(v, omega)

    def _find_lookahead(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if not self.path:
            return None

        # Find closest point on path
        best_idx = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(self.path):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i

        # Walk forward until we've accumulated look_ahead_dist
        accum = 0.0
        last_x, last_y = self.path[best_idx]
        for j in range(best_idx + 1, len(self.path)):
            px, py = self.path[j]
            step = math.hypot(px - last_x, py - last_y)
            accum += step
            if accum >= self.look_ahead_dist:
                return (px, py)
            last_x, last_y = px, py

        # If path is too short, just aim at final goal
        return self.path[-1]

    def _publish_cmd(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
