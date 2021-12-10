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

    σ_u = 0.05  # 0.01 # [rad/s] 正規分布ノイズの標準偏差
    σ_v = 0.05  # 0.01 # [rad/s] 正規分布ノイズの標準偏差
    σ_r = 0.05  # 0.01 # [rad/s] 正規分布ノイズの標準偏差
    σ_x = 0.5  # [m] 正規分布ノイズの標準偏差
    σ_y = 0.5  # [m] 正規分布ノイズの標準偏差
    σ_z = 0.05  # 0.01 # [rad] 正規分布ノイズの標準偏差

    x0 = 0
    y0 = 0
    psi0 = 0

    def __init__(self, publish_address="/ship1/sensor1", timer_period=0.1):
        """init."""
        super().__init__("simulated_sensor")
        self.delta_time = timer_period
        self.pub_sensor1 = self.create_publisher(Twist, publish_address, 1)
        self.pub_PositionSensor = self.create_publisher(
            PositionSensor, "/ship1/PositionSensor", 1
        )
        self.subscription = self.create_subscription(
            Twist, "/ship1/cmd_vel", self.listener_callback, 1
        )
        self.timer = self.create_timer(timer_period, self.sender_callback1)
        self.timer2 = self.create_timer(timer_period, self.sender_callback2)

    def sender_callback1(self):
        """sender_callback."""

        self.cmd_vel_Twist2.linear.x = self.cmd_vel_Twist.linear.x + np.random.normal(
            0, self.σ_u
        )  # センサ誤差を付加
        self.cmd_vel_Twist2.linear.y = self.cmd_vel_Twist.linear.y + np.random.normal(
            0, self.σ_v
        )  # センサ誤差を付加
        self.cmd_vel_Twist2.angular.z = self.cmd_vel_Twist.angular.z + np.random.normal(
            0, self.σ_r
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

        u_now = self.cmd_vel_Twist.linear.x
        v_now = self.cmd_vel_Twist.linear.y
        r_now = self.cmd_vel_Twist.angular.z

        self.sen_pose = self.get_pos(
            u_now,
            v_now,
            r_now,
            self.delta_time,
        )

        self.pub_PositionSensor.publish(self.sen_pose)
        self.get_logger().info(
            'MMG PositionSensor Publishing: "%s","%s","%s" '
            % (self.sen_pose.x, self.sen_pose.y, self.sen_pose.psi)
        )

    def get_pos(self, u_now, v_now, r_now, delta_time):
        pose = PositionSensor()

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

        pose.x = self.x0 + np.random.normal(0, self.σ_x)  # センサ誤差を付加
        pose.y = self.y0 + np.random.normal(0, self.σ_y)  # センサ誤差を付加
        pose.psi = self.psi0 + np.random.normal(0, self.σ_z)  # センサ誤差を付加

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
