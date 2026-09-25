import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_dir, 'maps', 'my_farm_map_nav2_v2.yaml')
    )

    return LaunchDescription([
        # Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ]),
            launch_arguments={'world': os.path.join(pkg_dir, 'worlds', 'farm.world')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ])
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'agri_robot', '-x', '0', '-y', '0', '-z', '0.25'],
            output='screen'
        ),

        # Nav2 with your saved map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            }.items()
        ),
    ])
