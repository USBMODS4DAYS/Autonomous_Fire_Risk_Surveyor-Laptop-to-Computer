from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # --- Arguments ---
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    # ✅ Global sim time parameter for all ROS2 nodes
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # --- Husky robot_state_publisher ---
    husky_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str)
    husky_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='husky',
        parameters=[{
            'robot_description': husky_description_content
        }]
    )
    ld.add_action(husky_state_pub)

    # --- Drone robot_state_publisher ---
    drone_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str)
    drone_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='drone',
        parameters=[{
            'robot_description': drone_description_content
        }]
    )
    ld.add_action(drone_state_pub)

    # --- EKF for Husky ---
    husky_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml'])
        ]
    )
    ld.add_action(husky_ekf)

    # --- EKF for Drone ---
    drone_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace='drone',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization_drone.yaml'])
        ]
    )
    ld.add_action(drone_ekf)

    # --- Drone SLAM (publishes drone/map -> drone/odom) ---
    drone_slam = GroupAction([
        PushRosNamespace('drone'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, 'drone_slam_toolbox.yaml'])
            ],
            remappings=[
                # Put the map topic & metadata under /drone/
                ('/map', '/drone/map'),
                ('/map_metadata', '/drone/map_metadata'),
                # (optional) if you ever subscribe with relative names:
                ('map', '/drone/map'),
                ('map_metadata', '/drone/map_metadata'),
            ]
        )
    ])
    ld.add_action(drone_slam)


    # --- Gazebo world setup ---
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # --- Spawn Husky ---
    husky_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', '/husky/robot_description',
                   '-name', 'husky', '-x', '0.0', '-y', '0.0', '-z', '0.4']
    )
    ld.add_action(husky_spawner)

    # --- Spawn Drone ---
    drone_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', '/drone/robot_description',
                   '-name', 'drone', '-x', '2.0', '-y', '0.0', '-z', '2.0']
    )
    ld.add_action(drone_spawner)

    # --- Bridge topics ---
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml'])}]
    )
    ld.add_action(gazebo_bridge)

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # --- Nav2 (for Husky only) ---
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
