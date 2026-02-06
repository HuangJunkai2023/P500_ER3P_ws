#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P500 ROS2 开发模式键盘控制: 发布 /cmd_vel (Twist)"""

import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("p500_ros2_keyboard")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear_vel = self.declare_parameter("linear", 0.3).value
        self.angular_vel = self.declare_parameter("angular", 0.6).value
        self.key_hold_timeout = 0.2
        self.last_key_time = {}

    def step(self):
        now = time.time()
        active = {k for k, t in self.last_key_time.items() if now - t <= self.key_hold_timeout}

        vx = 0.0
        omega = 0.0
        if 'w' in active:
            vx += self.linear_vel
        if 's' in active:
            vx -= self.linear_vel
        if 'a' in active:
            omega += self.angular_vel
        if 'd' in active:
            omega -= self.angular_vel

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = omega
        self.pub.publish(msg)


def realtime_keyboard_control():
    print("\n" + "=" * 60)
    print("P500 ROS2 模式实时键盘控制")
    print("=" * 60)
    print("W 前进 | S 后退 | A 左转 | D 右转 | Q 退出")
    print("按下移动，松开停止")
    print("=" * 60 + "\n")

    if not sys.stdin.isatty():
        print("[P500] 当前终端不是 TTY，无法实时按键控制。请用真实终端运行。")
        print("示例：source /opt/ros/<distro>/setup.bash && python p500_ros2_keyboard.py")
        return

    rclpy.init()
    node = KeyboardTeleop()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    print("\n退出程序...")
                    break
                if key in ('w', 'a', 's', 'd'):
                    node.last_key_time[key] = time.time()

            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
            time.sleep(0.05)  # 20Hz
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
        print("[P500] 已停止")


if __name__ == "__main__":
    realtime_keyboard_control()
