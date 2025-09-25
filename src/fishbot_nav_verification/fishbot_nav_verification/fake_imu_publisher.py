#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        
        # Create publisher for /imu/data topic
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )
        
        # Create timer for publishing at 50Hz
        self.timer = self.create_timer(0.02, self.publish_imu)
        
        self.get_logger().info('Fake IMU publisher started, publishing to /imu/data')
        
    def publish_imu(self):
        imu_msg = Imu()
        
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation (quaternion)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Set angular velocity (slight rotation for realism)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.1 * math.sin(time.time())
        
        # Set linear acceleration (gravity + noise)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81 + 0.1 * math.sin(time.time() * 2)
        
        # Set covariance matrices (identity for simplicity)
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeImuPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
