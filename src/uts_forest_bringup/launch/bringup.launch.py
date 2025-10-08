from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, TextSubstitution, LaunchConfiguration

def generate_launch_description():
    # choose teacher world: simple_trees or large_demo
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description="Teacher's base world to load"
    )

    # point GZ to our models (nice to have for any future model:// URIs)
    desc_share = FindPackageShare('uts_forest_description')
    models_dir = PathJoinSubstitution([desc_share, 'models'])
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, TextSubstitution(text=':'), EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]
    )

    # include teacher bringup
    teacher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('41068_ignition_bringup'),
                'launch', '41068_ignition.launch.py'
            ])
        ),
        launch_arguments={
            'slam': 'true',
            'nav2': 'true',
            'rviz': 'true',
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # absolute path to our model.sdf
    model_file = PathJoinSubstitution([desc_share, 'models', 'x3_uav', 'model.sdf'])

    # use Gazebo Sim spawner (ros_gz_sim)
    spawn_uav = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'x3_uav',
            '-file', model_file,
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        set_gz_path,
        teacher_launch,
        # wait a bit for the world to load, then spawn
        TimerAction(period=5.0, actions=[spawn_uav]),
    ])
