#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from math import radians, sin, cos
import time

class MinimalImuNode(Node):
    def __init__(self):
        super().__init__('hda536t_imu_node')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.yaw_off = self.get_parameter('yaw_offset_deg').get_parameter_value().double_value

        self.pub = self.create_publisher(Imu, 'imu/data', 50)
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.get_logger().info('Minimal IMU node up (publishing zeros; replace with full parser later)')

    def tick(self):
        t = time.time() - self.t0
        yaw = radians(self.yaw_off) + 0.1 * sin(0.5 * t)
        pitch = 0.0
        roll = 0.0

        cz, sz = cos(yaw * 0.5), sin(yaw * 0.5)
        cy, sy = cos(pitch * 0.5), sin(pitch * 0.5)
        cx, sx = cos(roll * 0.5), sin(roll * 0.5)
        w = cx * cy * cz + sx * sy * sz
        x = sx * cy * cz - cx * sy * sz
        y = cx * sy * cz + sx * cy * sz
        z = cx * cy * sz - sx * sy * cz

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id

        m.orientation.x = x; m.orientation.y = y; m.orientation.z = z; m.orientation.w = w
        m.orientation_covariance = [1e-3, 0.0, 0.0,
                                    0.0, 1e-3, 0.0,
                                    0.0, 0.0, 1e-3]

        m.angular_velocity.x = 0.0; m.angular_velocity.y = 0.0; m.angular_velocity.z = 0.0
        m.angular_velocity_covariance = [1e-3, 0.0, 0.0,
                                         0.0, 1e-3, 0.0,
                                         0.0, 0.0, 1e-3]

        m.linear_acceleration.x = 0.0; m.linear_acceleration.y = 0.0; m.linear_acceleration.z = 0.0
        m.linear_acceleration_covariance = [1e-2, 0.0, 0.0,
                                            0.0, 1e-2, 0.0,
                                            0.0, 0.0, 1e-2]

        self.pub.publish(m)

def main():
    rclpy.init()
    node = MinimalImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
