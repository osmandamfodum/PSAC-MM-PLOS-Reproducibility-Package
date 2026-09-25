import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    # ────────────────────────────────────────────────
    #   Package paths
    # ────────────────────────────────────────────────
    pkg_dir = get_package_share_directory('my_robot_description')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'farm.world')

    # Read URDF once
    robot_description = {'robot_description': open(urdf_file).read()}

    # ────────────────────────────────────────────────
    #   Launch arguments
    # ────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = LaunchConfiguration('world', default=world_file)
    use_gui = LaunchConfiguration('use_gui')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to SDF world file'
    )

    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch Gazebo GUI client; keep false for benchmark runs'
    )

    # ────────────────────────────────────────────────
    #   Robot State Publisher
    # ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # ────────────────────────────────────────────────
    #   Gazebo Classic – separate server + client + explicit plugins
    # ────────────────────────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world_arg,
            'verbose': 'true',
            'paused': 'false',
            'factory_plugin': '/opt/ros/humble/lib/libgazebo_ros_factory.so',
            'init_plugin': '/opt/ros/humble/lib/libgazebo_ros_init.so',
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ]),
        condition=IfCondition(use_gui),
    )

    # ────────────────────────────────────────────────
    #   Spawn robot
    # ────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'agri_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25',
            '-Y', '1.57',          # face toward positive X (crops)
            '-timeout', '30'
        ],
        output='both'
    )

    # ────────────────────────────────────────────────
    #   Final launch description
    # ────────────────────────────────────────────────
    return LaunchDescription([
        declare_sim_time,
        declare_world,
        declare_use_gui,

        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot,
    ])
