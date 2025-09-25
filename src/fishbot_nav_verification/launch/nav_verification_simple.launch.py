from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_nav_verification')
    fishbot_nav_share = get_package_share_directory('fishbot_nav')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(fishbot_nav_share, 'config', 'nav2_params.yaml')
    )
    declare_map_file = DeclareLaunchArgument(
        'map', default_value=os.path.join(fishbot_nav_share, 'map', 'map.yaml')
    )
    
    # Robot State Publisher (URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(fishbot_nav_share, 'urdf', 'wheeled_gazebo.urdf')]
    )
    
    # Static TF publishers for sensor frames
    static_tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.15','0','0.20','0','0','0','base_link','laser_link'],
             name='static_tf_laser'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.10','0','0','0','base_link','imu_link'],
             name='static_tf_imu'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.20','0','0.10','0','0','0','base_link','ultrasonic_front_link'],
             name='static_tf_us_front'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['-0.20','0','0.10','0','0','3.14159','base_link','ultrasonic_back_link'],
             name='static_tf_us_rear'),
    ]
    
    # Fake sensor data publishers
    fake_laser = Node(
        package='fishbot_nav_verification',
        executable='fake_laser_publisher',
        name='fake_laser_publisher',
        output='screen'
    )
    
    fake_imu = Node(
        package='fishbot_nav_verification',
        executable='fake_imu_publisher',
        name='fake_imu_publisher',
        output='screen'
    )
    
    fake_odom = Node(
        package='fishbot_nav_verification',
        executable='fake_odom_publisher',
        name='fake_odom_publisher',
        output='screen'
    )
    
    fake_ultrasonic = Node(
        package='fishbot_nav_verification',
        executable='fake_ultrasonic_publisher',
        name='fake_ultrasonic_publisher',
        output='screen'
    )
    
    # Command velocity logger
    cmd_vel_logger = Node(
        package='fishbot_nav_verification',
        executable='cmd_vel_logger',
        name='cmd_vel_logger',
        output='screen'
    )
    
    # Nav2 Core
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'use_sim_time': use_sim_time}]
    )
    
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )
    
    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )
    
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )
    
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server','amcl','planner_server','controller_server',
                                    'smoother_server','bt_navigator','waypoint_follower','behavior_server']}]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map_file)
    
    ld.add_action(robot_state_publisher)
    for tf_node in static_tf_nodes:
        ld.add_action(tf_node)
    
    ld.add_action(fake_laser)
    ld.add_action(fake_imu)
    ld.add_action(fake_odom)
    ld.add_action(fake_ultrasonic)
    ld.add_action(cmd_vel_logger)
    # 暂时注释掉EKF
    # ld.add_action(ekf)
    
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(planner)
    ld.add_action(controller)
    ld.add_action(smoother)
    ld.add_action(behavior)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager)
    
    return ld
