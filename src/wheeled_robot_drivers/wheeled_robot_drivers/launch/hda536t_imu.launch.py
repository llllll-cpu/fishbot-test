from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeled_robot_drivers',
            executable='hda536t_imu_node',
            name='hda536t_imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'crc_mode': 'none',
                'configure_on_startup': False,
                'continuous_divider': 20,
                'output_mask': 0x00000007
            }]
        )
    ])