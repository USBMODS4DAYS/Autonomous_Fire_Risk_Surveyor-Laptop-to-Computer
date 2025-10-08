from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forest_core',
            executable='avoid_lidar',
            name='avoid_lidar',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'cruise_speed': 0.8,
                'turn_speed': 1.0,
                'bias_gain': 0.3,
                'stop_dist': 1.5,
                'slow_dist': 2.5,
                'front_arc_deg': 60.0,
                'side_arc_deg': 70.0,
                'smooth_alpha': 0.6,
            }]
        )
    ])
