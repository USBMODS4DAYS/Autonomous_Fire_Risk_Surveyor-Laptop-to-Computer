from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="afr_planning",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/drone/map",
                        "goal_topic": "/drone/goal",
                        "map_frame": "drone/map",
                        "robot_frame": "drone_base_link",

                        "goal_altitude": 3.0,

                        # Frontier tuning:
                        "min_frontier_size": 15,          
                        "min_frontier_distance": 2.0,     
                        "frontier_clearance_cells": 2,    
                        "occ_threshold": 50,

                        "goal_reached_dist": 0.5,
                        "timer_period": 1.0,
                    }
                ]

            )
        ]
    )
