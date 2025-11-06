# # # #!/usr/bin/env python3

# # # import rclpy
# # # from rclpy.node import Node
# # # from nav_msgs.msg import Path
# # # from geometry_msgs.msg import PoseStamped
# # # import numpy as np
# # # from scipy.interpolate import CubicSpline
# # # from scipy.optimize import minimize
# # # import time

# # # class PathSmoother(Node):
# # #     def __init__(self):
# # #         super().__init__('path_smoother')
        
# # #         # Parameters
# # #         self.declare_parameter('max_deviation', 0.15)
# # #         self.declare_parameter('smoothing_factor', 0.5)
# # #         self.declare_parameter('shortcut_tolerance', 0.12)
# # #         self.declare_parameter('optimization_iterations', 50)
        
# # #         self.max_deviation = self.get_parameter('max_deviation').value
# # #         self.smoothing_factor = self.get_parameter('smoothing_factor').value
# # #         self.shortcut_tolerance = self.get_parameter('shortcut_tolerance').value
# # #         self.optimization_iterations = self.get_parameter('optimization_iterations').value
        
# # #         self.get_logger().info('Advanced Path Smoother node initialized')
        
# # #         # Subscriber
# # #         self.subscription = self.create_subscription(
# # #             Path,
# # #             '/plan',
# # #             self.path_callback,
# # #             10)
        
# # #         # Publisher
# # #         self.smoothed_path_pub = self.create_publisher(Path, '/smoothed_plan', 10)
        
# # #     def path_callback(self, msg):
# # #         """Callback khi nhận được đường A* từ Nav2"""
# # #         if len(msg.poses) < 3:
# # #             self.get_logger().warn('Path too short for smoothing')
# # #             return
            
# # #         self.get_logger().info(f'Received A* path with {len(msg.poses)} points')
        
# # #         # Extract points từ Path message
# # #         points = []
# # #         for pose_stamped in msg.poses:
# # #             x = pose_stamped.pose.position.x
# # #             y = pose_stamped.pose.position.y
# # #             points.append([x, y])
        
# # #         points = np.array(points)
        
# # #         try:
# # #             start_time = time.time()
            
# # #             # Sử dụng phương pháp đơn giản hơn trước
# # #             smoothed_points = self.simple_smooth_path(points)
            
# # #             smoothed_path = self.create_path_msg(smoothed_points, msg.header)
# # #             self.smoothed_path_pub.publish(smoothed_path)
            
# # #             processing_time = time.time() - start_time
# # #             self.get_logger().info(f'Published smoothed path with {len(smoothed_points)} points, processing time: {processing_time:.3f}s')
            
# # #         except Exception as e:
# # #             self.get_logger().error(f'Error in path smoothing: {str(e)}')
    
# # #     def simple_smooth_path(self, original_points):
# # #         """Phương pháp làm mịn đơn giản và ổn định hơn"""
# # #         if len(original_points) < 4:
# # #             return original_points
            
# # #         self.get_logger().info('Applying simple smoothing...')
        
# # #         # BƯỚC 1: Shortcut đơn giản
# # #         simplified = self.simple_shortcut(original_points)
# # #         self.get_logger().info(f'After shortcut: {len(simplified)} points')
        
# # #         # BƯỚC 2: Spline smoothing
# # #         smoothed = self.safe_spline_smoothing(simplified)
        
# # #         # BƯỚC 3: Constrain
# # #         final_path = self.safe_constrain(smoothed, original_points)
        
# # #         return final_path
    
# # #     def simple_shortcut(self, points):
# # #         """Shortcut đơn giản không có kiểm tra góc phức tạp"""
# # #         if len(points) < 4:
# # #             return points
            
# # #         result = [points[0]]
# # #         current_idx = 0
        
# # #         while current_idx < len(points) - 1:
# # #             best_idx = current_idx + 1
# # #             for target_idx in range(len(points)-1, current_idx + 1, -1):
# # #                 if self.is_safe_shortcut(points[current_idx], points[target_idx], 
# # #                                        points[current_idx+1:target_idx]):
# # #                     best_idx = target_idx
# # #                     break
                    
# # #             result.append(points[best_idx])
# # #             current_idx = best_idx
            
# # #         return np.array(result)
    
# # #     def is_safe_shortcut(self, point1, point2, intermediate_points):
# # #         """Kiểm tra shortcut đơn giản"""
# # #         if len(intermediate_points) == 0:
# # #             return True
            
# # #         line_vec = point2 - point1
# # #         line_length = np.linalg.norm(line_vec)
        
# # #         if line_length < 1e-6:
# # #             return True
            
# # #         line_vec_normalized = line_vec / line_length
        
# # #         max_distance = 0.0
# # #         for point in intermediate_points:
# # #             point_vec = point - point1
# # #             projection = np.dot(point_vec, line_vec_normalized)
            
# # #             if projection < 0 or projection > line_length:
# # #                 perpendicular_dist = min(
# # #                     np.linalg.norm(point - point1),
# # #                     np.linalg.norm(point - point2)
# # #                 )
# # #             else:
# # #                 closest_point = point1 + projection * line_vec_normalized
# # #                 perpendicular_dist = np.linalg.norm(point - closest_point)
                
# # #             max_distance = max(max_distance, perpendicular_dist)
            
# # #         return max_distance <= self.shortcut_tolerance
    
# # #     def safe_spline_smoothing(self, points):
# # #         """Spline an toàn với kiểm tra lỗi"""
# # #         if len(points) < 3:
# # #             return points
            
# # #         try:
# # #             distances = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
# # #             distances = np.insert(distances, 0, 0)
            
# # #             if distances[-1] < 1e-6:
# # #                 return points
                
# # #             normalized_distances = distances / distances[-1]
            
# # #             # Sử dụng ít điểm hơn để tránh over-smoothing
# # #             target_points = min(len(points) * 2, 100)
            
# # #             x_spline = CubicSpline(normalized_distances, points[:, 0])
# # #             y_spline = CubicSpline(normalized_distances, points[:, 1])
            
# # #             t_smooth = np.linspace(0, 1, target_points)
# # #             x_smooth = x_spline(t_smooth)
# # #             y_smooth = y_spline(t_smooth)
            
# # #             return np.column_stack((x_smooth, y_smooth))
            
# # #         except Exception as e:
# # #             self.get_logger().warn(f'Spline failed, using original: {e}')
# # #             return points
    
# # #     def safe_constrain(self, smoothed_points, original_points):
# # #         """Ràng buộc an toàn"""
# # #         if len(smoothed_points) == 0:
# # #             return original_points
            
# # #         constrained_points = []
        
# # #         for point in smoothed_points:
# # #             distances = np.linalg.norm(original_points - point, axis=1)
# # #             min_distance = np.min(distances)
# # #             nearest_idx = np.argmin(distances)
# # #             nearest_point = original_points[nearest_idx]
            
# # #             if min_distance > self.max_deviation:
# # #                 direction = nearest_point - point
# # #                 point = point + direction * 0.5  # Điều chỉnh 50%
                
# # #             constrained_points.append(point)
            
# # #         return np.array(constrained_points)
    
# # #     def create_path_msg(self, points, original_header):
# # #         """Tạo Path message từ các điểm"""
# # #         path_msg = Path()
# # #         path_msg.header = original_header
# # #         path_msg.header.stamp = self.get_clock().now().to_msg()
# # #         path_msg.header.frame_id = "map"
        
# # #         for point in points:
# # #             pose_stamped = PoseStamped()
# # #             pose_stamped.header = path_msg.header
# # #             pose_stamped.pose.position.x = float(point[0])
# # #             pose_stamped.pose.position.y = float(point[1])
# # #             pose_stamped.pose.position.z = 0.0
# # #             pose_stamped.pose.orientation.w = 1.0
# # #             path_msg.poses.append(pose_stamped)
            
# # #         return path_msg

# # # def main(args=None):
# # #     rclpy.init(args=args)
# # #     node = PathSmoother()
# # #     try:
# # #         rclpy.spin(node)
# # #     except KeyboardInterrupt:
# # #         node.get_logger().info('Path smoother node shutting down...')
# # #     finally:
# # #         node.destroy_node()
# # #         rclpy.shutdown()

# # # if __name__ == '__main__':
# # #     main()


# # #!/usr/bin/env python3

# # #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import numpy as np
# from scipy.interpolate import CubicSpline
# from scipy.optimize import minimize
# import time

# class PathSmoother(Node):
#     def __init__(self):
#         super().__init__('path_smoother')
        
#         # Parameters
#         self.declare_parameter('max_deviation', 0.3)
#         self.declare_parameter('shortcut_tolerance', 0.25)
#         self.declare_parameter('curvature_weight', 1.0)
#         self.declare_parameter('smoothness_weight', 0.8)
#         self.declare_parameter('constraint_weight', 0.1)
        
#         self.max_deviation = self.get_parameter('max_deviation').value
#         self.shortcut_tolerance = self.get_parameter('shortcut_tolerance').value
#         self.curvature_weight = self.get_parameter('curvature_weight').value
#         self.smoothness_weight = self.get_parameter('smoothness_weight').value
#         self.constraint_weight = self.get_parameter('constraint_weight').value
        
#         self.get_logger().info('Ultra Smooth Path Smoother node initialized')
        
#         # Subscriber
#         self.subscription = self.create_subscription(
#             Path,
#             '/plan',
#             self.path_callback,
#             10)
        
#         # Publisher
#         self.smoothed_path_pub = self.create_publisher(Path, '/smoothed_plan', 10)
        
#     def path_callback(self, msg):
#         """Callback khi nhận được đường A* từ Nav2"""
#         if len(msg.poses) < 3:
#             self.get_logger().warn('Path too short for smoothing')
#             return
            
#         self.get_logger().info(f'Received A* path with {len(msg.poses)} points')
        
#         # Extract points từ Path message
#         points = []
#         for pose_stamped in msg.poses:
#             x = pose_stamped.pose.position.x
#             y = pose_stamped.pose.position.y
#             points.append([x, y])
        
#         points = np.array(points)
        
#         try:
#             start_time = time.time()
            
#             # Áp dụng cả 2 phương pháp
#             smoothed_points = self.ultra_smooth_combination(points)
            
#             smoothed_path = self.create_path_msg(smoothed_points, msg.header)
#             self.smoothed_path_pub.publish(smoothed_path)
            
#             processing_time = time.time() - start_time
#             self.get_logger().info(f'Published ultra smooth path with {len(smoothed_points)} points, processing time: {processing_time:.3f}s')
            
#         except Exception as e:
#             self.get_logger().error(f'Error in path smoothing: {str(e)}')
    
#     def ultra_smooth_combination(self, original_points):
#         """Kết hợp mạnh mẽ 2 phương pháp: Short-cutting + Non-linear optimization"""
#         if len(original_points) < 4:
#             return original_points
            
#         self.get_logger().info('Applying ULTRA SMOOTH combination...')
        
#         # BƯỚC 1: AGGRESSIVE SHORT-CUTTING
#         simplified = self.aggressive_shortcut(original_points)
#         self.get_logger().info(f'After aggressive shortcut: {len(simplified)} points')
        
#         # BƯỚC 2: NON-LINEAR OPTIMIZATION FOR SMOOTHNESS
#         optimized = self.nonlinear_smooth_optimization(simplified, original_points)
        
#         return optimized
    
#     def aggressive_shortcut(self, points):
#         """Short-cutting mạnh mẽ - bỏ qua nhiều điểm trung gian"""
#         if len(points) < 4:
#             return points
            
#         result = [points[0]]  # Luôn giữ điểm đầu
        
#         i = 0
#         while i < len(points) - 1:
#             # Tìm điểm xa nhất có thể kết nối
#             farthest = i + 1
#             for j in range(len(points)-1, i, -1):
#                 if self.is_valid_shortcut(points[i], points[j], points[i+1:j]):
#                     farthest = j
#                     break
                    
#             result.append(points[farthest])
#             i = farthest
            
#         return np.array(result)
    
#     def is_valid_shortcut(self, p1, p2, intermediate_points):
#         """Kiểm tra shortcut có hợp lệ không"""
#         if len(intermediate_points) == 0:
#             return True
            
#         line_vec = p2 - p1
#         line_len = np.linalg.norm(line_vec)
        
#         if line_len < 1e-6:
#             return True
            
#         line_dir = line_vec / line_len
        
#         max_dist = 0.0
#         for point in intermediate_points:
#             vec_to_point = point - p1
#             projection = np.dot(vec_to_point, line_dir)
            
#             if projection < 0:
#                 dist = np.linalg.norm(point - p1)
#             elif projection > line_len:
#                 dist = np.linalg.norm(point - p2)
#             else:
#                 closest = p1 + projection * line_dir
#                 dist = np.linalg.norm(point - closest)
                
#             max_dist = max(max_dist, dist)
            
#         return max_dist <= self.shortcut_tolerance
    
#     def nonlinear_smooth_optimization(self, points, original_points):
#         """Tối ưu hóa phi tuyến để đạt độ mượt tối đa"""
#         if len(points) < 3:
#             return points
            
#         try:
#             # Khởi tạo biến tối ưu
#             x0 = points.flatten()
            
#             # Tạo ràng buộc
#             bounds = []
#             for i, point in enumerate(points):
#                 nearest_orig = self.find_nearest_original(point, original_points)
#                 bounds.extend([
#                     (nearest_orig[0] - self.max_deviation, nearest_orig[0] + self.max_deviation),
#                     (nearest_orig[1] - self.max_deviation, nearest_orig[1] + self.max_deviation)
#                 ])
            
#             # Tối ưu hóa
#             result = minimize(
#                 fun=self.smoothness_cost_function,
#                 x0=x0,
#                 args=(points, original_points),
#                 method='L-BFGS-B',
#                 bounds=bounds,
#                 options={'maxiter': 150, 'disp': False}
#             )
            
#             if result.success:
#                 optimized = result.x.reshape(-1, 2)
                
#                 # Áp dụng spline cuối cùng để đảm bảo độ mượt hoàn hảo
#                 final_smooth = self.ultra_smooth_spline(optimized)
#                 self.get_logger().info('Non-linear optimization successful')
#                 return final_smooth
#             else:
#                 self.get_logger().warn('Optimization failed, using spline only')
#                 return self.ultra_smooth_spline(points)
                
#         except Exception as e:
#             self.get_logger().warn(f'Optimization error: {e}')
#             return self.ultra_smooth_spline(points)
    
#     def smoothness_cost_function(self, x, original_optimized, original_points):
#         """Hàm chi phí cho độ mượt - trọng tâm vào giảm góc và độ cong"""
#         points = x.reshape(-1, 2)
#         cost = 0.0
        
#         # 1. Chi phí độ cong (quan trọng nhất)
#         curvature_cost = 0.0
#         for i in range(1, len(points) - 1):
#             v1 = points[i] - points[i-1]
#             v2 = points[i+1] - points[i]
            
#             if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
#                 v1_norm = v1 / np.linalg.norm(v1)
#                 v2_norm = v2 / np.linalg.norm(v2)
                
#                 dot = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
#                 angle = np.arccos(dot)
                
#                 # Penalize mạnh các góc lớn
#                 curvature_cost += angle * angle * angle  # Dùng mũ 3 để penalize mạnh hơn
        
#         # 2. Chi phí độ dài không đều
#         length_cost = 0.0
#         lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
#         if len(lengths) > 1:
#             length_std = np.std(lengths)
#             length_cost = length_std * 5.0
        
#         # 3. Chi phí khoảng cách đến đường gốc
#         distance_cost = 0.0
#         for point in points:
#             nearest = self.find_nearest_original(point, original_points)
#             dist = np.linalg.norm(point - nearest)
#             distance_cost += dist * dist
        
#         # Kết hợp chi phí với trọng số
#         total_cost = (self.curvature_weight * curvature_cost + 
#                      self.smoothness_weight * length_cost + 
#                      self.constraint_weight * distance_cost)
        
#         return total_cost
    
#     def find_nearest_original(self, point, original_points):
#         """Tìm điểm gần nhất trong đường gốc"""
#         distances = np.linalg.norm(original_points - point, axis=1)
#         return original_points[np.argmin(distances)]
    
#     def ultra_smooth_spline(self, points):
#         """Spline siêu mịn với số điểm tối ưu"""
#         if len(points) < 3:
#             return points
            
#         try:
#             # Tính khoảng cách tích lũy
#             distances = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
#             distances = np.insert(distances, 0, 0)
            
#             if distances[-1] < 1e-6:
#                 return points
                
#             normalized_dist = distances / distances[-1]
            
#             # Tạo nhiều điểm cho đường cực mịn
#             target_points = min(len(points) * 4, 400)
            
#             # Spline với điều kiện biên tự nhiên
#             x_spline = CubicSpline(normalized_dist, points[:, 0], bc_type='natural')
#             y_spline = CubicSpline(normalized_dist, points[:, 1], bc_type='natural')
            
#             t_new = np.linspace(0, 1, target_points)
#             x_new = x_spline(t_new)
#             y_new = y_spline(t_new)
            
#             return np.column_stack((x_new, y_new))
            
#         except Exception as e:
#             self.get_logger().warn(f'Spline failed: {e}')
#             return points
    
#     def create_path_msg(self, points, original_header):
#         """Tạo Path message từ các điểm"""
#         path_msg = Path()
#         path_msg.header = original_header
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
        
#         for point in points:
#             pose_stamped = PoseStamped()
#             pose_stamped.header = path_msg.header
#             pose_stamped.pose.position.x = float(point[0])
#             pose_stamped.pose.position.y = float(point[1])
#             pose_stamped.pose.position.z = 0.0
#             pose_stamped.pose.orientation.w = 1.0
#             path_msg.poses.append(pose_stamped)
            
#         return path_msg

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSmoother()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Path smoother node shutting down...')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize
import time

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        
        # === CÁC THAM SỐ ĐÃ ĐƯỢC ĐIỀU CHỈNH ===
        
        # Giảm độ "hung hăng" khi cắt góc và tối ưu
        # Giảm khoảng cách tối đa mà đường đi mượt được phép lệch khỏi đường A*
        self.declare_parameter('max_deviation', 0.15)      # Giảm từ 0.3
        # Giảm khoảng cách tối đa cho phép khi "cắt góc" (shortcut)
        self.declare_parameter('shortcut_tolerance', 0.1)  # Giảm từ 0.25
        
        # Giữ nguyên trọng số cho độ cong (quan trọng để mượt)
        self.declare_parameter('curvature_weight', 1.0)
        
        # Giảm trọng số độ mượt (độ dài), ưu tiên cho ràng buộc
        self.declare_parameter('smoothness_weight', 0.5)   # Giảm từ 0.8
        
        # TĂNG MẠNH trọng số bám đường gốc (QUAN TRỌNG NHẤT)
        # Buộc thuật toán tối ưu phải bám sát đường A* an toàn
        self.declare_parameter('constraint_weight', 1.5)   # Tăng mạnh từ 0.1
        
        # ======================================
        
        self.max_deviation = self.get_parameter('max_deviation').value
        self.shortcut_tolerance = self.get_parameter('shortcut_tolerance').value
        self.curvature_weight = self.get_parameter('curvature_weight').value
        self.smoothness_weight = self.get_parameter('smoothness_weight').value
        self.constraint_weight = self.get_parameter('constraint_weight').value
        
        self.get_logger().info('Ultra Smooth Path Smoother (Tuned Params) initialized')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        
        # Publisher
        self.smoothed_path_pub = self.create_publisher(Path, '/smoothed_plan', 10)
        
    def path_callback(self, msg):
        """Callback khi nhận được đường A* từ Nav2"""
        if len(msg.poses) < 3:
            self.get_logger().warn('Path too short for smoothing')
            return
            
        self.get_logger().info(f'Received A* path with {len(msg.poses)} points')
        
        # Extract points từ Path message
        points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            points.append([x, y])
        
        points = np.array(points)
        
        try:
            start_time = time.time()
            
            # Áp dụng cả 2 phương pháp
            smoothed_points = self.ultra_smooth_combination(points)
            
            smoothed_path = self.create_path_msg(smoothed_points, msg.header)
            self.smoothed_path_pub.publish(smoothed_path)
            
            processing_time = time.time() - start_time
            self.get_logger().info(f'Published ultra smooth path with {len(smoothed_points)} points, processing time: {processing_time:.3f}s')
            
        except Exception as e:
            self.get_logger().error(f'Error in path smoothing: {str(e)}')
    
    def ultra_smooth_combination(self, original_points):
        """Kết hợp mạnh mẽ 2 phương pháp: Short-cutting + Non-linear optimization"""
        if len(original_points) < 4:
            return original_points
            
        self.get_logger().info('Applying ULTRA SMOOTH combination...')
        
        # BƯỚC 1: AGGRESSIVE SHORT-CUTTING
        simplified = self.aggressive_shortcut(original_points)
        self.get_logger().info(f'After aggressive shortcut: {len(simplified)} points')
        
        # BƯỚC 2: NON-LINEAR OPTIMIZATION FOR SMOOTHNESS
        optimized = self.nonlinear_smooth_optimization(simplified, original_points)
        
        return optimized
    
    def aggressive_shortcut(self, points):
        """Short-cutting mạnh mẽ - bỏ qua nhiều điểm trung gian"""
        if len(points) < 4:
            return points
            
        result = [points[0]]  # Luôn giữ điểm đầu
        
        i = 0
        while i < len(points) - 1:
            # Tìm điểm xa nhất có thể kết nối
            farthest = i + 1
            for j in range(len(points)-1, i, -1):
                if self.is_valid_shortcut(points[i], points[j], points[i+1:j]):
                    farthest = j
                    break
                    
            result.append(points[farthest])
            i = farthest
            
        return np.array(result)
    
    def is_valid_shortcut(self, p1, p2, intermediate_points):
        """Kiểm tra shortcut có hợp lệ không"""
        if len(intermediate_points) == 0:
            return True
            
        line_vec = p2 - p1
        line_len = np.linalg.norm(line_vec)
        
        if line_len < 1e-6:
            return True
            
        line_dir = line_vec / line_len
        
        max_dist = 0.0
        for point in intermediate_points:
            vec_to_point = point - p1
            projection = np.dot(vec_to_point, line_dir)
            
            if projection < 0:
                dist = np.linalg.norm(point - p1)
            elif projection > line_len:
                dist = np.linalg.norm(point - p2)
            else:
                closest = p1 + projection * line_dir
                dist = np.linalg.norm(point - closest)
                
            max_dist = max(max_dist, dist)
            
        return max_dist <= self.shortcut_tolerance
    
    def nonlinear_smooth_optimization(self, points, original_points):
        """Tối ưu hóa phi tuyến để đạt độ mượt tối đa"""
        if len(points) < 3:
            return points
            
        try:
            # Khởi tạo biến tối ưu
            x0 = points.flatten()
            
            # Tạo ràng buộc
            bounds = []
            for i, point in enumerate(points):
                nearest_orig = self.find_nearest_original(point, original_points)
                bounds.extend([
                    (nearest_orig[0] - self.max_deviation, nearest_orig[0] + self.max_deviation),
                    (nearest_orig[1] - self.max_deviation, nearest_orig[1] + self.max_deviation)
                ])
            
            # Tối ưu hóa
            result = minimize(
                fun=self.smoothness_cost_function,
                x0=x0,
                args=(points, original_points),
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 150, 'disp': False}
            )
            
            if result.success:
                optimized = result.x.reshape(-1, 2)
                
                # Áp dụng spline cuối cùng để đảm bảo độ mượt hoàn hảo
                final_smooth = self.ultra_smooth_spline(optimized)
                self.get_logger().info('Non-linear optimization successful')
                return final_smooth
            else:
                self.get_logger().warn('Optimization failed, using spline only')
                return self.ultra_smooth_spline(points)
                
        except Exception as e:
            self.get_logger().warn(f'Optimization error: {e}')
            return self.ultra_smooth_spline(points)
    
    def smoothness_cost_function(self, x, original_optimized, original_points):
        """Hàm chi phí cho độ mượt - trọng tâm vào giảm góc và độ cong"""
        points = x.reshape(-1, 2)
        cost = 0.0
        
        # 1. Chi phí độ cong (quan trọng nhất)
        curvature_cost = 0.0
        for i in range(1, len(points) - 1):
            v1 = points[i] - points[i-1]
            v2 = points[i+1] - points[i]
            
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                v1_norm = v1 / np.linalg.norm(v1)
                v2_norm = v2 / np.linalg.norm(v2)
                
                dot = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
                angle = np.arccos(dot)
                
                # Penalize mạnh các góc lớn
                curvature_cost += angle * angle * angle  # Dùng mũ 3 để penalize mạnh hơn
        
        # 2. Chi phí độ dài không đều
        length_cost = 0.0
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if len(lengths) > 1:
            length_std = np.std(lengths)
            length_cost = length_std * 5.0
        
        # 3. Chi phí khoảng cách đến đường gốc
        distance_cost = 0.0
        for point in points:
            nearest = self.find_nearest_original(point, original_points)
            dist = np.linalg.norm(point - nearest)
            distance_cost += dist * dist
        
        # Kết hợp chi phí với trọng số
        total_cost = (self.curvature_weight * curvature_cost + 
                      self.smoothness_weight * length_cost + 
                      self.constraint_weight * distance_cost)
        
        return total_cost
    
    def find_nearest_original(self, point, original_points):
        """Tìm điểm gần nhất trong đường gốc"""
        distances = np.linalg.norm(original_points - point, axis=1)
        return original_points[np.argmin(distances)]
    
    def ultra_smooth_spline(self, points):
        """Spline siêu mịn với số điểm tối ưu"""
        if len(points) < 3:
            return points
            
        try:
            # Tính khoảng cách tích lũy
            distances = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
            distances = np.insert(distances, 0, 0)
            
            if distances[-1] < 1e-6:
                return points
                
            normalized_dist = distances / distances[-1]
            
            # Tạo nhiều điểm cho đường cực mịn
            target_points = min(len(points) * 4, 400)
            
            # Spline với điều kiện biên tự nhiên
            x_spline = CubicSpline(normalized_dist, points[:, 0], bc_type='natural')
            y_spline = CubicSpline(normalized_dist, points[:, 1], bc_type='natural')
            
            t_new = np.linspace(0, 1, target_points)
            x_new = x_spline(t_new)
            y_new = y_spline(t_new)
            
            return np.column_stack((x_new, y_new))
            
        except Exception as e:
            self.get_logger().warn(f'Spline failed: {e}')
            return points
    
    def create_path_msg(self, points, original_header):
        """Tạo Path message từ các điểm"""
        path_msg = Path()
        path_msg.header = original_header
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
            
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Path smoother node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()