from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('afr_planning')
    mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_yaml],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    planner = Node(
        package='afr_planning',
        executable='astar_planner',
        name='astar_planner',
        output='screen'
    )

    return LaunchDescription([twist_mux, planner])
