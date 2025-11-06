#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time
from math import sin, cos

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # ===== THAY ĐỔI TỌA ĐỘ CÁC WAYPOINTS Ở ĐÂY =====
        # Danh sách các waypoints (x, y, yaw)
        # yaw: góc quay (radians) - 0: hướng đông, 1.57: hướng bắc, 3.14: hướng tây, -1.57: hướng nam
        self.waypoints = [
    {'name': 'Point_A', 'x': 5.0, 'y': 5.0, 'yaw': 0.0},     # Gần hơn
    {'name': 'Point_B', 'x': -5.0, 'y': -5.0, 'yaw': 0.0},
    
]
        # ================================================
        
        # Tạo Action Client để gửi goal đến Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.current_waypoint_index = 0
        self.goal_handle = None
        
        self.get_logger().info('==============================================')
        self.get_logger().info('🚀 Waypoint Navigator Node Initialized')
        self.get_logger().info(f'📍 Total waypoints: {len(self.waypoints)}')
        for wp in self.waypoints:
            self.get_logger().info(f'   - {wp["name"]}: ({wp["x"]:.2f}, {wp["y"]:.2f})')
        self.get_logger().info('==============================================')
        
        # Đợi Nav2 action server sẵn sàng
        self.get_logger().info('⏳ Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('✅ Nav2 action server is ready!')
        
        # Bắt đầu chạy sau 3 giây
        self.get_logger().info('⏰ Starting navigation in 3 seconds...')
        self.timer = self.create_timer(3.0, self.start_navigation)
    
    def start_navigation(self):
        """Bắt đầu điều hướng đến waypoint đầu tiên"""
        self.timer.cancel()  # Hủy timer sau lần chạy đầu
        self.send_next_waypoint()
    
    def send_next_waypoint(self):
        """Gửi waypoint tiếp theo đến Nav2"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('')
            self.get_logger().info('==============================================')
            self.get_logger().info('🎉 ALL WAYPOINTS COMPLETED! 🎉')
            self.get_logger().info('==============================================')
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        self.get_logger().info('')
        self.get_logger().info('==============================================')
        self.get_logger().info(f'🎯 Navigating to {waypoint["name"]} [{self.current_waypoint_index + 1}/{len(self.waypoints)}]')
        self.get_logger().info(f'   📌 Position: x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}')
        self.get_logger().info(f'   🧭 Orientation: yaw={waypoint["yaw"]:.2f} rad')
        self.get_logger().info('==============================================')
        
        # Tạo goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(
            waypoint['x'], 
            waypoint['y'], 
            waypoint['yaw']
        )
        
        # Gửi goal đến Nav2
        self.get_logger().info('📤 Sending goal to Nav2...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def create_pose(self, x, y, yaw):
        """Tạo PoseStamped từ tọa độ x, y, yaw"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Chuyển yaw sang quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        
        return pose
    
    def goal_response_callback(self, future):
        """Callback khi Nav2 chấp nhận hoặc từ chối goal"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2!')
            self.get_logger().error('   Please check if the goal is in valid space')
            return
        
        self.get_logger().info('✅ Goal accepted by Nav2')
        self.get_logger().info('🚗 Robot is moving...')
        
        # Đợi kết quả
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback nhận feedback từ Nav2 trong quá trình di chuyển"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # Tính khoảng cách đến mục tiêu
        waypoint = self.waypoints[self.current_waypoint_index]
        dx = waypoint['x'] - current_pose.position.x
        dy = waypoint['y'] - current_pose.position.y
        distance = (dx**2 + dy**2)**0.5
        
        self.get_logger().info(
            f'🚗 Position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) | '
            f'Distance to goal: {distance:.2f}m',
            throttle_duration_sec=2.0  # Log mỗi 2 giây
        )
    
    def result_callback(self, future):
        """Callback khi hoàn thành waypoint"""
        result = future.result().result
        status = future.result().status
        
        waypoint = self.waypoints[self.current_waypoint_index]
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('')
            self.get_logger().info('==============================================')
            self.get_logger().info(f'✅ Successfully reached {waypoint["name"]}!')
            self.get_logger().info('==============================================')
            
            # Chuyển sang waypoint tiếp theo
            self.current_waypoint_index += 1
            
            # Đợi 2 giây trước khi chuyển waypoint
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info('⏸️  Waiting 2 seconds before next waypoint...')
                time.sleep(2.0)
            
            self.send_next_waypoint()
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('')
            self.get_logger().error('==============================================')
            self.get_logger().error(f'❌ Failed to reach {waypoint["name"]}')
            self.get_logger().error('   Status: ABORTED (Path blocked or invalid)')
            self.get_logger().error('==============================================')
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('')
            self.get_logger().warn('==============================================')
            self.get_logger().warn(f'⚠️  Navigation to {waypoint["name"]} was canceled')
            self.get_logger().warn('==============================================')
            
        else:
            self.get_logger().error(f'❌ Unknown status: {status}')

def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('')
        navigator.get_logger().info('==============================================')
        navigator.get_logger().info('🛑 Waypoint Navigator shutting down...')
        navigator.get_logger().info('==============================================')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()