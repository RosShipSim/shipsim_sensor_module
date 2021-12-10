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

    def __init__(self):
        """init."""
        super().__init__("sensor", namespace="ship1")
        self.declare_parameter("publish_address", "/ship1/obs_vel")
        self.declare_parameter("subscribe_address", "/ship1/cmd_vel")
        self.declare_parameter("delta_time", 0.1)

        self.declare_parameter("mu_r", 0.00)  # [rad/s] mean of Gauss noize
        self.declare_parameter("sigma_r", 0.05)  # [rad/s] std of Gauss noize

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Twist, publish_address, 1)

        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            Twist, subscribe_address, self.listener_callback, 1
        )

        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""

        μ_r = self.get_parameter("mu_r").value
        σ_r = self.get_parameter("sigma_r").value

        self.cmd_vel_Twist2.linear.x = self.cmd_vel_Twist.linear.x
        self.cmd_vel_Twist2.angular.z = self.cmd_vel_Twist.angular.z + np.random.normal(
            μ_r, σ_r
        )

        self.publisher.publish(self.cmd_vel_Twist2)
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
