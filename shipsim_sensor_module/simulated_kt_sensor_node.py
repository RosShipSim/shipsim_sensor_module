#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class SensorNode(Node):
    """SensorNode."""

    cmd_vel_Twist = Twist()
    cmd_vel_Twist2 = Twist()

    def __init__(self, publish_address="/ship1/sensor", timer_period=0.1):
        """init."""
        super().__init__("simulated_sensor")
        self.delta_time = timer_period
        self.pub_sensor = self.create_publisher(Twist, publish_address, 1)
        self.subscription = self.create_subscription(
            Twist, "/ship1/cmd_vel", self.listener_callback, 1
        )
        self.timer = self.create_timer(timer_period, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""

        σ_r = 0.05  # 0.01 #0.1 [rad/s] 正規分布ノイズの標準偏差

        self.cmd_vel_Twist2.linear.x = self.cmd_vel_Twist.linear.x
        self.cmd_vel_Twist2.angular.z = self.cmd_vel_Twist.angular.z + np.random.normal(
            0, self.σ_r
        )  # 回頭角速度にセンサ誤差を付加

        self.pub_sensor.publish(self.cmd_vel_Twist2)
        self.get_logger().info(
            'KT SensorNode Publishing: "%s","%s" '
            % (self.cmd_vel_Twist2.linear.x, self.cmd_vel_Twist2.angular.z)
        )

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'KT SensorNode heard: "%s","%s" ' % (msg.linear.x, msg.angular.z)
        )
        self.cmd_vel_Twist.linear.x = msg.linear.x
        self.cmd_vel_Twist.angular.z = msg.angular.z


def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
