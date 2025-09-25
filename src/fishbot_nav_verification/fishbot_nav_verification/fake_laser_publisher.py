#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_publisher')
        
        # Create publisher for /scan topic
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        
        # Create timer for publishing at 10Hz
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Fake laser publisher started, publishing to /scan')
        
    def publish_scan(self):
        scan_msg = LaserScan()
        
        # Set header
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_link'
        
        # Set scan parameters
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        
        # Generate fake scan data (simple room-like environment)
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        scan_msg.ranges = []
        
        for i in range(num_readings):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Create a simple rectangular room (5m x 3m)
            if abs(math.cos(angle)) > 0.1:  # Front/back walls
                range_val = 2.5 / abs(math.cos(angle))
            else:  # Left/right walls
                range_val = 1.5 / abs(math.sin(angle))
            
            # Add some noise and obstacles
            range_val += 0.1 * math.sin(time.time() + angle * 10)
            
            # Clamp to valid range
            range_val = max(scan_msg.range_min, min(scan_msg.range_max, range_val))
            scan_msg.ranges.append(range_val)
        
        self.scan_publisher.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
