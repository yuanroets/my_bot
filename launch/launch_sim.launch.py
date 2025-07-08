import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_my_bot = FindPackageShare('my_bot')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    world_sdf = '/usr/share/gz/gz-sim8/worlds/empty.sdf'
    
    # Get the robot description from rsp.launch.py
    pkg_path = get_package_share_directory('my_bot')
    controllers_config = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'drive_bot.rviz')

    return LaunchDescription([
        # Include robot state publisher with sim time
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_my_bot, 'launch', 'rsp.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': f'-r -s {world_sdf}'}.items(),
        ),
        
        # Wait a bit then spawn robot in Gazebo
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', 'robot_description', '-name', 'bot_name'],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controllers_config,
                {'use_sim_time': True}
            ],
            output='screen',
        ),
        
        # Spawn joint state broadcaster first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Then spawn differential drive controller
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
        
        # Launch RViz
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),
    ])