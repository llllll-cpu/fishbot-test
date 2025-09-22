'''
作者: 小鱼 (改进版)
描述: 真机 URDF + Nav2 + RViz 启动（实时跟随真机 TF/joint_states）
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 真机场景：use_sim_time false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', 
        default=os.path.join(fishbot_navigation2_dir, 'maps', 'map.yaml')
    )
    nav2_param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'param', 'wheeled_real.yaml')
    )

    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    urdf_file = os.path.join(fishbot_description_dir, 'urdf', 'wheeled_gazebo.urdf')  # 根据实际路径改

    # 读取 URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),

        # robot_state_publisher：用真机 joint_states/TF 渲染 URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # 如果真机没有 joint_states，取消上面注释启用：
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),

        # 启动 Nav2（真机参数）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),

        # RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
