from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fishbot_fourwheel_odom',
            executable='fourwheel_diff_odom',
            name='fourwheel_diff_odom',
            output='screen',
            parameters=[{
                'wheel_radius': 0.05,         # m
                'wheel_base':   0.30,         # m
                'odom_frame':   'odom',
                'base_frame':   'base_link',
                'joint_state_topic': '/joint_states',
                'front_left_joint':  'front_left_wheel_joint',
                'rear_left_joint':   'rear_left_wheel_joint',
                'front_right_joint': 'front_right_wheel_joint',
                'rear_right_joint':  'rear_right_wheel_joint',
                'encoder_is_radians': True,   # 若用 tick 改为 False 并设置 ticks_per_rev
                'ticks_per_rev': 2048,
                'publish_tf': True,
            }],
        )
    ])
