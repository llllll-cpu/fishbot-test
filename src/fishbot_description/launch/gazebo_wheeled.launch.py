
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 自定义模型路径
#os.environ['GAZEBO_MODEL_PATH'] = '/home/ros/fishbot/src/fishbot_description/models:' + os.environ.get('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')
os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models')


def generate_launch_description():
    # robot_name_in_model = 'fishbot'
    # package_name = 'fishbot_description'
    # urdf_name = "fishbot_gazebo.urdf"
    
    robot_name_in_model = 'wheeled_robot'
    package_name = 'fishbot_description'
    urdf_name = "wheeled_gazebo.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    # 使用占据栅格生成的 OGM 世界
    # gazebo_world_path = os.path.join(pkg_share, 'world/fishbot.world')
    # gazebo_world_path = os.path.join(pkg_share, 'world/chejian_world.world')
    #gazebo_world_path = '/home/ros/fishbot/src/fishbot_description/world/ogm_world.world'
    gazebo_world_path = os.path.join(pkg_share, 'world/empty.world')

    # Start Gazebo server
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
    #     output='screen')
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=['gzserver', '--verbose', gazebo_world_path, '-s', 'libgazebo_ros_factory.so'],
    #     output='screen')

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')
        
    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            # 初始位姿，可按需要调整
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen')
	
    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{'use_sim_time': True}]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path]
        )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_rviz_cmd)


    return ld
