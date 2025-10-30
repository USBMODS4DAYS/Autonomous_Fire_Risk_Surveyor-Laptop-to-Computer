from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('afr_planning')

    # arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use sim time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # paths
    mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    drone_ekf_yaml = os.path.join(pkg_share, 'config', 'robot_localization_drone.yaml')

    # 1) EKF for the drone (publishes drone/map -> drone/odom -> drone/base_link)
    drone_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace='drone',
        name='robot_localization',
        output='screen',
        parameters=[
            drone_ekf_yaml,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2) twist_mux just for the drone (optional but keeping your structure)
    # NOTE: we remap mux output to /drone/cmd_vel so it ultimately goes to Gazebo VelocityControl
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace='drone',
        name='twist_mux',
        output='screen',
        parameters=[mux_yaml,
                    {'use_sim_time': use_sim_time}],
        remappings=[
            # mux output velocity -> Gazebo bridge input
            ('/cmd_vel_out', '/drone/cmd_vel'),

            # you can also remap whatever your teleop topic is
            # e.g. ('cmd_vel_teleop', '/drone/teleop_cmd'),
            # plus planner will feed /drone/cmd_vel_nav
        ]
    )

    # 3) A* planner node for the drone
    #    - namespace='drone' means all relative topics become /drone/...
    #    - we tell it which TF frames to use
    #    - we remap its cmd_vel_nav -> /drone/cmd_vel_nav so twist_mux can listen
    planner = Node(
        package='afr_planning',
        executable='astar_planner',
        namespace='drone',
        name='astar_planner',
        output='screen',
        parameters=[{
            'map_frame': 'drone/map',
            'base_link_frame': 'drone/base_link',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # use Husky's global map topic for now
            # (the SLAM node in your big bringup is probably publishing /map)
            ('map', '/map'),

            # goal topic we’ll click in RViz for the drone
            ('goal_pose', '/drone/goal_pose'),

            # send nav command to mux channel expected as /drone/cmd_vel_nav
            ('cmd_vel_nav', '/drone/cmd_vel_nav'),
        ]
    )

    # NOTE: for twist_mux.yaml you will want an input for cmd_vel_nav
    # that forwards to cmd_vel_out. Something like:
    #  - name: planner
    #    topic: cmd_vel_nav
    #    timeout: 0.5
    #    priority: 50
    #
    # Because of namespace='drone', inside mux that becomes /drone/cmd_vel_nav

    return LaunchDescription([
        use_sim_arg,
        drone_ekf,
        twist_mux,
        planner
    ])
