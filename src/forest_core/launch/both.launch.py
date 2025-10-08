# launch/both.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='large_demo')
    use_rviz  = DeclareLaunchArgument('rviz',  default_value='true')
    use_nav2  = DeclareLaunchArgument('nav2',  default_value='true')

    teacher_share = FindPackageShare('41068_ignition_bringup')

    # 1) Start teacher bringup (Husky + world + optional Nav2/RViz)
    husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([teacher_share, 'launch', '41068_ignition.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'slam':  'true',
            'nav2':  LaunchConfiguration('nav2'),
            'rviz':  LaunchConfiguration('rviz'),
        }.items()
    )

    # 2) Spawn the drone (parrot) into the same sim
    # Use the xacro directly; GZ will expand it. Choose spawn height with -z.
    parrot_xacro = PathJoinSubstitution([teacher_share, 'urdf_drone', 'parrot.urdf.xacro'])
    spawn_parrot = ExecuteProcess(
        cmd=[
            'ros2','run','ros_gz_sim','create',
            '-name','parrot',
            '-file', parrot_xacro,
            '-x','0','-y','0','-z','2.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg, use_rviz, use_nav2,
        husky,
        TimerAction(period=5.0, actions=[spawn_parrot]),  # wait for world then spawn
    ])
