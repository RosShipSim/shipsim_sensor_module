#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import PositionSensor


class SensorNode(Node):
    """SensorNode."""

    cmd_vel_Twist = Twist()
    cmd_vel_Twist2 = Twist()
    sen_pose = PositionSensor()

    x0 = 0
    y0 = 0
    psi0 = 0

    def __init__(self):
        """init."""
        super().__init__("sensor", namespace="ship1")
        self.declare_parameter("publish_address", "/ship1/obs_vel")
        self.declare_parameter("subscribe_address", "/ship1/cmd_vel")
        self.declare_parameter("delta_time", 0.1)

        self.declare_parameter("mu_u", 0.00)  # [m/s] mean of Gauss noize
        self.declare_parameter("sigma_u", 0.05)  # [m/s] std of Gauss noize
        self.declare_parameter("mu_v", 0.00)  # [m/s] mean of Gauss noize
        self.declare_parameter("sigma_v", 0.05)  # [m/s] std of Gauss noize
        self.declare_parameter("mu_r", 0.00)  # [rad/s] mean of Gauss noize
        self.declare_parameter("sigma_r", 0.05)  # [rad/s] std of Gauss noize

        self.declare_parameter("mu_x", 0.00)  # [m] mean of Gauss noize
        self.declare_parameter("sigma_x", 0.5)  # [m] std of Gauss noize
        self.declare_parameter("mu_y", 0.00)  # [m] mean of Gauss noize
        self.declare_parameter("sigma_y", 0.5)  # [m] std of Gauss noize
        self.declare_parameter("mu_psi", 0.00)  # [rad] mean of Gauss noize
        self.declare_parameter("sigma_psi", 0.05)  # [rad] std of Gauss noize

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_sensor1 = self.create_publisher(Twist, publish_address, 1)

        self.pub_PositionSensor = self.create_publisher(
            PositionSensor, "/ship1/PositionSensor", 1
        )

        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            Twist, subscribe_address, self.listener_callback, 1
        )

        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback1)
        self.timer2 = self.create_timer(delta_time, self.sender_callback2)

    def sender_callback1(self):
        """sender_callback."""

        μ_u = self.get_parameter("mu_u").value
        σ_u = self.get_parameter("sigma_u").value
        μ_v = self.get_parameter("mu_v").value
        σ_v = self.get_parameter("sigma_v").value
        μ_r = self.get_parameter("mu_r").value
        σ_r = self.get_parameter("sigma_r").value

        self.cmd_vel_Twist2.linear.x = self.cmd_vel_Twist.linear.x + np.random.normal(
            μ_u, σ_u
        )  # センサ誤差を付加
        self.cmd_vel_Twist2.linear.y = self.cmd_vel_Twist.linear.y + np.random.normal(
            μ_v, σ_v
        )  # センサ誤差を付加
        self.cmd_vel_Twist2.angular.z = self.cmd_vel_Twist.angular.z + np.random.normal(
            μ_r, σ_r
        )  # センサ誤差を付加

        self.pub_sensor1.publish(self.cmd_vel_Twist2)
        self.get_logger().info(
            'MMG Sensor1 Publishing: "%s","%s","%s" '
            % (
                self.cmd_vel_Twist2.linear.x,
                self.cmd_vel_Twist2.linear.y,
                self.cmd_vel_Twist2.angular.z,
            )
        )

    def sender_callback2(self):
        """sender_callback."""
        delta_time = self.get_parameter("delta_time").value

        u_now = self.cmd_vel_Twist.linear.x
        v_now = self.cmd_vel_Twist.linear.y
        r_now = self.cmd_vel_Twist.angular.z

        self.sen_pose = self.get_pos(
            u_now,
            v_now,
            r_now,
            delta_time,
        )

        self.pub_PositionSensor.publish(self.sen_pose)
        self.get_logger().info(
            'MMG PositionSensor Publishing: "%s","%s","%s" '
            % (self.sen_pose.x, self.sen_pose.y, self.sen_pose.psi)
        )

    def get_pos(self, u_now, v_now, r_now, delta_time):
        pose = PositionSensor()

        μ_x = self.get_parameter("mu_x").value
        σ_x = self.get_parameter("sigma_x").value
        μ_y = self.get_parameter("mu_y").value
        σ_y = self.get_parameter("sigma_y").value
        μ_psi = self.get_parameter("mu_psi").value
        σ_psi = self.get_parameter("sigma_psi").value

        self.x0 = (
            self.x0
            + (u_now * delta_time * np.cos(self.psi0))
            - (v_now * delta_time * np.sin(self.psi0))
        )
        self.y0 = (
            self.y0
            + (v_now * delta_time * np.cos(self.psi0))
            + (u_now * delta_time * np.sin(self.psi0))
        )
        self.psi0 = self.psi0 + r_now * delta_time
        if self.psi0 > np.pi:
            self.psi0 = self.psi0 - 2 * np.pi
        elif self.psi0 < (-np.pi):
            self.psi0 = self.psi0 + 2 * np.pi

        pose.x = self.x0 + np.random.normal(μ_x, σ_x)  # センサ誤差を付加
        pose.y = self.y0 + np.random.normal(μ_y, σ_y)  # センサ誤差を付加
        pose.psi = self.psi0 + np.random.normal(μ_psi, σ_psi)  # センサ誤差を付加

        return pose

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'MMG SensorNode heard: "%s","%s","%s"  '
            % (msg.linear.x, msg.linear.y, msg.angular.z)
        )
        self.cmd_vel_Twist.linear.x = msg.linear.x
        self.cmd_vel_Twist.linear.y = msg.linear.y
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
