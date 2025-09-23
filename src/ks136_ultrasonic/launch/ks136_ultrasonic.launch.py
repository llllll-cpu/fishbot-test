from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ks136_ultrasonic',
            executable='ks136_ultrasonic_node',
            name='ks136_ultrasonic',
            output='screen',
            parameters=[
                {'mock_mode': True},   # 先用模拟跑通；有硬件后设为 False
                {'i2c_bus': 1},
                {'i2c_addr_8bit': 0xE8},
                {'front_cmd_mm': 0x30},  # 前：探头5，返回 mm
                {'rear_cmd_mm':  0x40},  # 后：探头7，返回 mm
                {'min_range_m': 0.13},
                {'max_range_m': 4.5},
                {'fov_deg': 60.0},
                {'front_frame_id': 'ultra_front'},
                {'rear_frame_id':  'ultra_rear'},
            ]
        )
    ])
