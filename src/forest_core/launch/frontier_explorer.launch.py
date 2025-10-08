from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forest_core',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'odom_topic': '/odom',
                'use_cmd_vel': True,
                'cmd_vel_topic': '/cmd_vel',
                'goal_topic': '/goal_pose',
                'linear_speed_max': 1.0,
                'angular_speed_max': 1.2,
                'linear_k': 0.6,
                'angular_k': 1.2,
                'frontier_min_size': 15,
                'target_reached_dist': 0.6,
            }]
        )
    ])
