#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
import time

class FakeUltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('fake_ultrasonic_publisher')
        
        # Create publishers for ultrasonic topics
        self.front_publisher = self.create_publisher(
            Range,
            '/ultrasonic/front',
            10
        )
        
        self.rear_publisher = self.create_publisher(
            Range,
            '/ultrasonic/rear',
            10
        )
        
        # Create timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_ranges)
        
        self.get_logger().info('Fake ultrasonic publisher started')
        
    def publish_ranges(self):
        # Front ultrasonic
        front_msg = Range()
        front_msg.header.stamp = self.get_clock().now().to_msg()
        front_msg.header.frame_id = 'ultrasonic_front_link'
        front_msg.radiation_type = Range.ULTRASOUND
        front_msg.field_of_view = math.radians(30.0)  # 30 degrees
        front_msg.min_range = 0.02
        front_msg.max_range = 4.0
        
        # Simulate obstacle detection (simple wall at 2m)
        front_msg.range = 2.0 + 0.1 * math.sin(time.time())
        
        # Rear ultrasonic
        rear_msg = Range()
        rear_msg.header.stamp = self.get_clock().now().to_msg()
        rear_msg.header.frame_id = 'ultrasonic_back_link'
        rear_msg.radiation_type = Range.ULTRASOUND
        rear_msg.field_of_view = math.radians(30.0)  # 30 degrees
        rear_msg.min_range = 0.02
        rear_msg.max_range = 4.0
        
        # Simulate obstacle detection (simple wall at 1.5m)
        rear_msg.range = 1.5 + 0.1 * math.cos(time.time())
        
        self.front_publisher.publish(front_msg)
        self.rear_publisher.publish(rear_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeUltrasonicPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
