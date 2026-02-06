#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P500 ROS 开发模式键盘控制: 发布 /cmd_vel (Twist)"""

import sys
import time
import termios
import tty
import select

import rospy
from geometry_msgs.msg import Twist


def realtime_keyboard_control():
    print("\n" + "=" * 60)
    print("P500 ROS 模式实时键盘控制")
    print("=" * 60)
    print("W 前进 | S 后退 | A 左转 | D 右转 | Q 退出")
    print("按下移动，松开停止")
    print("=" * 60 + "\n")

    rospy.init_node("p500_ros_keyboard", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    linear_vel = rospy.get_param("~linear", 0.3)
    angular_vel = rospy.get_param("~angular", 0.6)
    key_hold_timeout = 0.2

    if not sys.stdin.isatty():
        print("[P500] 当前终端不是 TTY，无法实时按键控制。请用真实终端运行。")
        print("示例：source /opt/ros/noetic/setup.bash && python p500_ros_keyboard.py")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    last_key_time = {}

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    print("\n退出程序...")
                    break
                if key in ('w', 'a', 's', 'd'):
                    last_key_time[key] = time.time()

            now = time.time()
            active = {k for k, t in last_key_time.items() if now - t <= key_hold_timeout}

            vx = 0.0
            omega = 0.0
            if 'w' in active:
                vx += linear_vel
            if 's' in active:
                vx -= linear_vel
            if 'a' in active:
                omega += angular_vel
            if 'd' in active:
                omega -= angular_vel

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = omega
            pub.publish(msg)

            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # 停车
        stop = Twist()
        pub.publish(stop)
        print("[P500] 已停止")


if __name__ == "__main__":
    realtime_keyboard_control()
