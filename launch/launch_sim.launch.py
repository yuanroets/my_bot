from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_my_bot = FindPackageShare('my_bot')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    world_sdf = PathJoinSubstitution([pkg_my_bot, 'worlds', 'empty.sdf'])

    return LaunchDescription([
        # Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_my_bot, 'launch', 'rsp.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        # Gazebo Harmonic (headless)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': f'-r -s {world_sdf}'}.items(),
        ),
        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'bot_name'],
            output='screen',
        ),
    ])