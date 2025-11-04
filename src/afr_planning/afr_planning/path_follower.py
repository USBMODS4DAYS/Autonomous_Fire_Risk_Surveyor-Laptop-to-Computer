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

# ---------- small helpers ----------

def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(x, y, z, w) -> float:
    """Extract yaw (around Z) from a quaternion."""
    # standard yaw formula
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")

        # Frames / topics
        self.map_frame = "drone/map"
        self.base_frame = "drone_base_link"
        self.cmd_vel_topic = "/drone/cmd_vel"
        self.path_topic = "/drone/astar_path"

        # Parameters (you can tweak from command line later)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("look_ahead_dist", 0.6)
        self.declare_parameter("max_lin_speed", 0.6)
        self.declare_parameter("max_ang_speed", 1.0)
        self.declare_parameter("goal_xy_tolerance", 0.2)
        self.declare_parameter("yaw_tolerance", 0.1)

        self.control_dt = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.look_ahead = float(self.get_parameter("look_ahead_dist").value)
        self.max_v = float(self.get_parameter("max_lin_speed").value)
        self.max_w = float(self.get_parameter("max_ang_speed").value)
        self.goal_tol = float(self.get_parameter("goal_xy_tolerance").value)
        self.yaw_tol = float(self.get_parameter("yaw_tolerance").value)

        # TF buffer / listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sub / pub
        self.path_sub = self.create_subscription(Path, self.path_topic, self._on_path, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Internal state
        self.current_path: List[Tuple[float, float]] = []
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.current_wp_idx: int = 0
        self.active: bool = False

        # Control timer
        self.timer = self.create_timer(self.control_dt, self._control_loop)

        self.get_logger().info(
            "Path follower ready. It will follow /drone/astar_path using /drone/cmd_vel."
        )

    # ---------- path callback ----------
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty path, stopping.")
            self.current_path = []
            self.goal_xy = None
            self.active = False
            self._publish_zero()
            return

        self.current_path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.goal_xy = self.current_path[-1]
        self.current_wp_idx = 0
        self.active = True

        self.get_logger().info(
            f"New path received: {len(self.current_path)} points. "
            f"Goal = ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})"
        )

    # ---------- control loop ----------
    def _control_loop(self):
        if not self.active or not self.current_path:
            # nothing to do
            return

        # Get pose of drone in map frame
        pose = self._get_pose()
        if pose is None:
            self._publish_zero()
            return

        x, y, yaw = pose

        # Check if we are at goal
        gx, gy = self.goal_xy
        dist_to_goal = math.hypot(gx - x, gy - y)
        if dist_to_goal < self.goal_tol:
            # close enough → stop
            self.get_logger().info("Reached goal (within tolerance), stopping.")
            self.active = False
            self._publish_zero()
            return


        # Choose a look-ahead target on the path
        target_idx = self.current_wp_idx
        best_idx = target_idx
        best_dist = 0.0

        for i in range(self.current_wp_idx, len(self.current_path)):
            px, py = self.current_path[i]
            d = math.hypot(px - x, py - y)
            if d < self.look_ahead:
                best_idx = i
                best_dist = d
            else:
                # first point beyond lookahead distance – good carrot
                best_idx = i
                best_dist = d
                break

        self.current_wp_idx = best_idx
        tx, ty = self.current_path[best_idx]

        # Heading to target
        target_yaw = math.atan2(ty - y, tx - x)
        yaw_err = angle_wrap(target_yaw - yaw)

        # Simple P controller
        k_v = 0.8
        k_w = 1.5

        # If yaw error large, rotate in place first
        twist = Twist()
        if abs(yaw_err) > self.yaw_tol * 3.0:
            twist.linear.x = 0.0
            twist.angular.z = max(-self.max_w, min(self.max_w, k_w * yaw_err))
        else:
            # Move forward with some speed, scaled by distance & heading
            v = k_v * best_dist * math.cos(yaw_err)
            v = max(0.0, min(self.max_v, v))
            w = k_w * yaw_err
            w = max(-self.max_w, min(self.max_w, w))

            twist.linear.x = v
            twist.angular.z = w

        self.cmd_pub.publish(twist)

    # ---------- helpers ----------
    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn_throttle(
                2000, f"TF {self.map_frame}->{self.base_frame} not ready: {e}"
            )
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)
        return (t.x, t.y, yaw)

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
