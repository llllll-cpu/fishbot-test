#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Cmd_vel logger started, monitoring /cmd_vel')
        
    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f"CMD_VEL: linear=[{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}] "
            f"angular=[{msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
