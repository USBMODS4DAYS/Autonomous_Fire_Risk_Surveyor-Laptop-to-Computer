#!/usr/bin/env python3
# ~/41068_ws/src/path_smoother/path_smoother/waypoint_runner.py

import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os

class WaypointRunner(Node):

    def __init__(self):
        super().__init__('waypoint_runner')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # === THÊM DÒNG NÀY ===
        self.total_waypoints = 0
        # ======================
        
        self.get_logger().info('Waypoint Runner node initialized.')
        self.run()

    def run(self):
        pkg_path = get_package_share_directory('path_smoother')
        yaml_file = os.path.join(pkg_path, 'config', 'my_waypoints.yaml')
        
        self.get_logger().info(f'Loading waypoints from {yaml_file}')
        
        try:
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                if 'waypoints' not in data:
                    self.get_logger().error("File YAML không có key 'waypoints'")
                    return
                waypoints = data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Không thể đọc file YAML: {str(e)}')
            return

        self.get_logger().info('Waiting for "navigate_through_poses" action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []
        self.get_logger().info(f'Sending {len(waypoints)} waypoints to Nav2...')

        for point in waypoints:
            x, y = point
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            goal_msg.poses.append(pose)

        # === THÊM DÒNG NÀY ===
        # Lưu lại tổng số điểm để dùng trong feedback
        self.total_waypoints = len(goal_msg.poses)
        # ======================

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :) Waypoints are being processed...')
        get_result_future = self.goal_handle.get_result_async()
        
        # Chờ cho đến khi kết quả hoàn thành
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # Kiểm tra kết quả
        result = get_result_future.result().result
        if result:
            self.get_logger().info('Finished navigating all waypoints!')
        else:
            self.get_logger().warn('Goal failed or was cancelled.')
        
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # === SỬA LẠI TOÀN BỘ HÀM NÀY ===
        try:
            feedback = feedback_msg.feedback
            # Tính toán điểm hiện tại
            current_waypoint_num = self.total_waypoints - feedback.number_of_poses_remaining
            self.get_logger().info(f'Navigating... Reached waypoint {current_waypoint_num}/{self.total_waypoints}. ' +
                                   f'({feedback.number_of_poses_remaining} remaining)',
                                   throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().warn(f'Error in feedback_callback: {e}')
        # ================================

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointRunner()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()

if __name__ == '__main__':
    main()