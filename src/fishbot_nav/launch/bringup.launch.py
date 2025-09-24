from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_nav')
    rslidar_share = get_package_share_directory('rslidar_sdk')
    imu_share = get_package_share_directory('wheeled_robot_drivers')
    odom_share = get_package_share_directory('fishbot_fourwheel_odom')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    map_file     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    )
    declare_map_file = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_share, 'map', 'map.yaml')
    )

    # Robot State Publisher (URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(pkg_share, 'urdf', 'wheeled_gazebo.urdf')]
    )

    # #（可选）静态TF——如果你的驱动已内置URDF并发布TF，可注释掉这几条
    # static_tf_nodes = [
    #     Node(package='tf2_ros', executable='static_transform_publisher',
    #          arguments=['0.15','0','0.20','0','0','0','base_link','laser_frame'],
    #          name='static_tf_laser'),
    #     Node(package='tf2_ros', executable='static_transform_publisher',
    #          arguments=['0','0','0.10','0','0','0','base_link','imu_link'],
    #          name='static_tf_imu'),
    #     Node(package='tf2_ros', executable='static_transform_publisher',
    #          arguments=['0.20','0','0.10','0','0','0','base_link','ultrasonic_front'],
    #          name='static_tf_us_front'),
    #     Node(package='tf2_ros', executable='static_transform_publisher',
    #          arguments=['-0.20','0','0.10','0','0','3.14159','base_link','ultrasonic_rear'],
    #          name='static_tf_us_rear'),
    # ]

    # Robot Localization EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

    # LiDAR driver (Robosense) -> /rslidar_points (frame: laser_link)
    rslidar = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': os.path.join(rslidar_share, 'config', 'config.yaml')}]
    )

    # PointCloud2 -> LaserScan (/scan) for AMCL and costmaps
    pcl2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/rslidar_points'), ('scan', '/scan')],
        parameters=[
            {'target_frame': 'laser_link'},
            {'transform_tolerance': 0.01},
            {'min_height': -1.0},
            {'max_height': 1.0},
            {'range_min': 0.2},
            {'range_max': 30.0},
            {'scan_time': 0.1},
            {'use_inf': True},
            {'concurrency_level': 1},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.004363323}  # ~0.25 deg
        ]
    )

    # IMU node (can also IncludeLaunchDescription, but direct Node allows overrides)
    imu_node = Node(
        package='wheeled_robot_drivers',
        executable='hda536t_imu_node',
        name='hda536t_imu',
        output='screen',
        parameters=[{'frame_id': 'imu_link'}]
    )

    # Four-wheel odometry
    odom_node = Node(
        package='fishbot_fourwheel_odom',
        executable='fourwheel_diff_odom',
        name='fourwheel_diff_odom',
        output='screen'
    )

    # Ultrasonic front/rear → sensor_msgs/Range topics for RangeLayer
    ultrasonic_node = Node(
        package='ks136_ultrasonic',
        executable='ks136_ultrasonic_node',
        name='ks136_ultrasonic',
        output='screen',
        parameters=[
            {'mock_mode': True},
            {'front_frame_id': 'ultrasonic_front_link'},
            {'rear_frame_id':  'ultrasonic_back_link'},
            {'min_range_m': 0.13},
            {'max_range_m': 4.5},
            {'fov_deg': 60.0}
        ]
    )

    # Nav2 Core（使用 lifecycle_manager 自动拉起）
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
    # for n in static_tf_nodes:
    #     ld.add_action(n)
    ld.add_action(rslidar)
    ld.add_action(pcl2scan)
    ld.add_action(imu_node)
    ld.add_action(odom_node)
    ld.add_action(ultrasonic_node)
    ld.add_action(ekf)

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


