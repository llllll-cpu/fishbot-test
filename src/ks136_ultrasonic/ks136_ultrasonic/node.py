#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None


class KS136(Node):
    """
    KS136 I2C 超声波前/后双路 → 发布两路 sensor_msgs/Range：
      /ultrasonic/front, /ultrasonic/rear
    适配 Nav2 RangeSensorLayer；支持 mock_mode 无硬件联调。
    """

    def __init__(self):
        super().__init__('ks136_ultrasonic')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr_8bit', 0xE8)  # 8-bit 地址 => 7-bit 0x74
        self.declare_parameter('front_cmd_mm', 0x30)   # 单探头返回 mm：探头5=0x30，6=0x38，7=0x40，8=0x48...
        self.declare_parameter('rear_cmd_mm',  0x40)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('min_range_m', 0.13)
        self.declare_parameter('max_range_m', 4.5)
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('front_frame_id', 'ultra_front')
        self.declare_parameter('rear_frame_id',  'ultra_rear')
        self.declare_parameter('mock_mode', True)      # 默认 True：先跑通

        self.bus_num = int(self.get_parameter('i2c_bus').value)
        self.addr_8  = int(self.get_parameter('i2c_addr_8bit').value)
        self.addr_7  = self.addr_8 >> 1
        self.cmd_f   = int(self.get_parameter('front_cmd_mm').value)
        self.cmd_r   = int(self.get_parameter('rear_cmd_mm').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.min_r   = float(self.get_parameter('min_range_m').value)
        self.max_r   = float(self.get_parameter('max_range_m').value)
        self.fov_rad = float(self.get_parameter('fov_deg').value) * 3.1415926535 / 180.0
        self.frame_f = str(self.get_parameter('front_frame_id').value)
        self.frame_r = str(self.get_parameter('rear_frame_id').value)
        self.mock    = bool(self.get_parameter('mock_mode').value)

        self.pub_f = self.create_publisher(Range, 'ultrasonic/front', 10)
        self.pub_r = self.create_publisher(Range, 'ultrasonic/rear',  10)

        if not self.mock:
            if SMBus is None:
                self.get_logger().error('缺少 smbus2：请 pip install smbus2，或先启用 mock_mode:=true')
                raise RuntimeError('smbus2 missing')
            self.bus = SMBus(self.bus_num)
            self.get_logger().info(f'I2C /dev/i2c-{self.bus_num}, 7-bit addr=0x{self.addr_7:02X}')
        else:
            self.bus = None
            self.get_logger().warn('mock_mode 开启：使用模拟距离发布 Range。')

        self.timer = self.create_timer(max(1.0/self.rate_hz, 0.01), self._on_timer)

    # ---- KS136 低级 I2C ----
    def _write_cmd(self, cmd:int):
        self.bus.write_byte_data(self.addr_7, 0x02, cmd)

    def _read_u16_r2_r3(self) -> int:
        hi = self.bus.read_byte_data(self.addr_7, 0x02)
        lo = self.bus.read_byte_data(self.addr_7, 0x03)
        return ((hi & 0xFF) << 8) | (lo & 0xFF)

    def _measure_m(self, cmd:int) -> Optional[float]:
        try:
            self._write_cmd(cmd)
            time.sleep(0.05)  # 50ms
            mm = self._read_u16_r2_r3()
            return mm / 1000.0
        except Exception as e:
            self.get_logger().warn(f'I2C error: {e}')
            return None

    # ---- 发布 ----
    def _publish_range(self, pub, frame_id: str, dist_m: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov_rad
        msg.min_range = self.min_r
        msg.max_range = self.max_r
        if dist_m < self.min_r:
            dist_m = self.min_r
        elif dist_m > self.max_r:
            dist_m = float('inf')  # Nav2 range_layer 视为无障碍
        msg.range = dist_m
        pub.publish(msg)

    def _on_timer(self):
        if self.mock:
            self._publish_range(self.pub_f, self.frame_f, 0.50)
            self._publish_range(self.pub_r, self.frame_r, 0.80)
            return
        d_f = self._measure_m(self.cmd_f)
        if d_f and d_f > 0:
            self._publish_range(self.pub_f, self.frame_f, d_f)
        d_r = self._measure_m(self.cmd_r)
        if d_r and d_r > 0:
            self._publish_range(self.pub_r, self.frame_r, d_r)


def main():
    rclpy.init()
    node = KS136()
    try:
        rclpy.spin(node)
    finally:
        if getattr(node, 'bus', None):
            try: node.bus.close()
            except Exception: pass
        node.destroy_node()
        rclpy.shutdown()
