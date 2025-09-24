#!/usr/bin/env python3
import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    # roll=pitch=0
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class FourWheelDiffOdom(Node):
    def __init__(self):
        super().__init__('fourwheel_diff_odom')

        # ---------------- Params ----------------
        self.declare_parameter('wheel_radius', 0.05)        # m
        self.declare_parameter('wheel_base',   0.30)        # m (左右轮心距)
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('joint_state_topic', '/joint_states')

        # 关节名
        self.declare_parameter('front_left_joint',  'front_left_wheel_joint')
        self.declare_parameter('rear_left_joint',   'rear_left_wheel_joint')
        self.declare_parameter('front_right_joint', 'front_right_wheel_joint')
        self.declare_parameter('rear_right_joint',  'rear_right_wheel_joint')

        # 输入是否为弧度（true=弧度；false=tick 需要 ticks_per_rev）
        self.declare_parameter('encoder_is_radians', True)
        self.declare_parameter('ticks_per_rev', 2048)

        # 卷积/噪声相关（可调）
        self.declare_parameter('publish_tf', True)

        # 读取参数
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base   = float(self.get_parameter('wheel_base').value)
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.publish_tf   = bool(self.get_parameter('publish_tf').value)

        self.joint_names = {
            'fl': self.get_parameter('front_left_joint').value,
            'rl': self.get_parameter('rear_left_joint').value,
            'fr': self.get_parameter('front_right_joint').value,
            'rr': self.get_parameter('rear_right_joint').value,
        }

        self.encoder_is_radians = bool(self.get_parameter('encoder_is_radians').value)
        self.ticks_per_rev      = float(self.get_parameter('ticks_per_rev').value)

        # ---------------- State ----------------
        self.last_pos: Dict[str, Optional[float]] = {k: None for k in ['fl','rl','fr','rr']}
        self.last_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ---------------- Pub/Sub ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        joint_topic = self.get_parameter('joint_state_topic').value
        self.sub = self.create_subscription(JointState, joint_topic, self.cb_joint, qos)

        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_lr_js = self.create_publisher(JointState, 'left_right_joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'FourWheelDiffOdom started. Listen: {joint_topic}')
        self.get_logger().info(f'FL/FR/RL/RR joints: {self.joint_names}')
        self.get_logger().info(f'wheel_radius={self.wheel_radius} m, wheel_base={self.wheel_base} m')

    # 将 position（弧度或tick） -> 轮外周累计位移（米）
    def pos_to_travel_m(self, pos: float) -> float:
        if self.encoder_is_radians:
            # 弧度 * r = 弧长
            return pos * self.wheel_radius
        else:
            # tick -> 转数 -> 弧长
            rev = pos / self.ticks_per_rev
            return (2.0 * math.pi * self.wheel_radius) * rev

    def cb_joint(self, msg: JointState):
        # 把四个轮各自的 position 取出来
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        present = {}
        for key, jname in self.joint_names.items():
            idx = name_to_idx.get(jname, None)
            if idx is None or idx >= len(msg.position):
                self.get_logger().warn_once(f'Missing joint "{jname}" in JointState; waiting...')
                return
            present[key] = float(msg.position[idx])

        # 时间
        t = msg.header.stamp
        t_now = self.get_clock().now().to_msg() if (t.sec == 0 and t.nanosec == 0) else t

        # 初始化：只记录首次位置
        if any(self.last_pos[k] is None for k in self.last_pos):
            for k in self.last_pos:
                self.last_pos[k] = present[k]
            self.last_time = t_now
            return

        # 计算每个轮的增量（米）
        dl_fl = self.pos_to_travel_m(present['fl'] - self.last_pos['fl'])
        dl_rl = self.pos_to_travel_m(present['rl'] - self.last_pos['rl'])
        dr_fr = self.pos_to_travel_m(present['fr'] - self.last_pos['fr'])
        dr_rr = self.pos_to_travel_m(present['rr'] - self.last_pos['rr'])

        # 左右对合并（同侧两轮取平均，适配四轮差速结构）
        d_left  = 0.5 * (dl_fl + dl_rl)
        d_right = 0.5 * (dr_fr + dr_rr)

        # 保存本次位置为下次基准
        for k in self.last_pos:
            self.last_pos[k] = present[k]
        self.last_time = t_now

        # 差速里程计积分
        ds   = 0.5 * (d_left + d_right)
        dth  = (d_right - d_left) / self.wheel_base

        if abs(dth) < 1e-9:
            # 近似直线
            dx = ds * math.cos(self.yaw)
            dy = ds * math.sin(self.yaw)
        else:
            # 圆弧运动
            R = ds / dth
            dx = R * (math.sin(self.yaw + dth) - math.sin(self.yaw))
            dy = -R * (math.cos(self.yaw + dth) - math.cos(self.yaw))

        self.x   += dx
        self.y   += dy
        self.yaw = (self.yaw + dth + math.pi) % (2.0 * math.pi) - math.pi

        # 线、角速度（用 dt 估计）
        # 若 joint_state header 时间不可用，上面 t_now 用 node clock 时间
        dt = (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9) \
             if (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0) else None
        # 更稳妥：用 rclpy now
        dt = None  # 这里用 None，下面不发布 twist 的真实速度（可按需加 timer）

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = t_now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(self.yaw)[2], yaw_to_quat(self.yaw)[3]
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 可根据实际给些经验协方差
        odom.pose.covariance = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-2,
]
        self.pub_odom.publish(odom)

        # 发布 TF
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = t_now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        # 发布左右合并的 JointState 供调试
        js = JointState()
        js.header.stamp = t_now
        js.name = ['left_pair', 'right_pair']
        # 用累计位移（米）当作 position（仅供观察），更严谨可单独发布自定义消息
        js.position = [self.pos_to_travel_m(present['fl']), self.pos_to_travel_m(present['fr'])]
        self.pub_lr_js.publish(js)

    # end class


def main():
    rclpy.init()
    node = FourWheelDiffOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
