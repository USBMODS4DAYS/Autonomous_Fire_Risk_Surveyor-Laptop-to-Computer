#!/usr/bin/env python3
from typing import Tuple, List, Optional
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AvoidLidar(Node):
    def __init__(self):
        super().__init__('avoid_lidar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cruise_speed', 0.8)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('bias_gain', 0.3)
        self.declare_parameter('stop_dist', 1.5)
        self.declare_parameter('slow_dist', 2.5)
        self.declare_parameter('front_arc_deg', 60.0)
        self.declare_parameter('side_arc_deg', 70.0)
        self.declare_parameter('smooth_alpha', 0.6)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.latest_scan: Optional[LaserScan] = None
        self.prev_cmd: Optional[Twist] = None
        self.timer = self.create_timer(0.1, self.tick)

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def tick(self):
        if self.latest_scan is None:
            return
        cruise = float(self.get_parameter('cruise_speed').value)
        turn = float(self.get_parameter('turn_speed').value)
        bias_k = float(self.get_parameter('bias_gain').value)
        stop_d = float(self.get_parameter('stop_dist').value)
        slow_d = float(self.get_parameter('slow_dist').value)
        front_min, left_min, right_min = self.sector_minima(self.latest_scan)
        lin = cruise
        ang = 0.0
        if front_min < stop_d:
            lin = 0.0
            ang = turn if right_min > left_min else -turn
        elif front_min < slow_d:
            lin = 0.4 * cruise
            if abs(right_min - left_min) > 0.15:
                ang = bias_k * (1.0 if right_min > left_min else -1.0)
        else:
            if abs(right_min - left_min) > 0.5:
                ang = 0.2 * bias_k * (1.0 if right_min > left_min else -1.0)
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        cmd = self._smooth(cmd)
        self.pub_cmd.publish(cmd)

    def sector_minima(self, scan: LaserScan) -> Tuple[float, float, float]:
        front_arc = math.radians(float(self.get_parameter('front_arc_deg').value))
        side_arc = math.radians(float(self.get_parameter('side_arc_deg').value))

        def sector_min(center_angle: float, half_width: float) -> float:
            a0 = center_angle - half_width
            a1 = center_angle + half_width
            i0 = max(0, int((a0 - scan.angle_min) / scan.angle_increment))
            i1 = min(len(scan.ranges) - 1, int((a1 - scan.angle_min) / scan.angle_increment))
            vals: List[float] = []
            for d in scan.ranges[i0:i1 + 1]:
                if scan.range_min < d < scan.range_max and math.isfinite(d):
                    vals.append(d)
            return min(vals) if vals else float('inf')

        front_min = sector_min(0.0, front_arc * 0.5)
        left_min = sector_min(+math.pi / 2.0, side_arc * 0.5)
        right_min = sector_min(-math.pi / 2.0, side_arc * 0.5)
        return front_min, left_min, right_min

    def _smooth(self, cmd: Twist) -> Twist:
        alpha = float(self.get_parameter('smooth_alpha').value)
        if self.prev_cmd is None or alpha >= 0.999:
            self.prev_cmd = cmd
            return cmd
        out = Twist()
        out.linear.x = alpha * cmd.linear.x + (1.0 - alpha) * self.prev_cmd.linear.x
        out.angular.z = alpha * cmd.angular.z + (1.0 - alpha) * self.prev_cmd.angular.z
        self.prev_cmd = out
        return out

def main():
    rclpy.init()
    node = AvoidLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
