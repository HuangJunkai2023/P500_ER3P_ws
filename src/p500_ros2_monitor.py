#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P500 ROS2 监控：订阅 /odom 和 /battery 并打印状态"""

import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


class P500Ros2Monitor(Node):
    def __init__(self):
        super().__init__("p500_ros2_monitor")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.batt_topic = self.declare_parameter("battery_topic", "/battery").value
        self.print_interval = float(self.declare_parameter("print_interval", 0.5).value)

        self.last_print = 0.0
        self.last_odom = None
        self.last_batt = None

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(BatteryState, self.batt_topic, self.on_battery, 10)

        self.get_logger().info(
            f"订阅: {self.odom_topic} (Odometry), {self.batt_topic} (BatteryState)"
        )

    def on_odom(self, msg: Odometry):
        self.last_odom = msg
        self.maybe_print()

    def on_battery(self, msg: BatteryState):
        self.last_batt = msg
        self.maybe_print()

    def maybe_print(self):
        now = time.time()
        if now - self.last_print < self.print_interval:
            return
        self.last_print = now

        parts = []
        if self.last_odom is not None:
            vx = self.last_odom.twist.twist.linear.x
            vy = self.last_odom.twist.twist.linear.y
            wz = self.last_odom.twist.twist.angular.z
            parts.append(f"odom: vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}")
        if self.last_batt is not None:
            v = self.last_batt.voltage
            pct = self.last_batt.percentage
            if pct is not None and pct >= 0:
                parts.append(f"battery: {v:.2f}V {pct*100:.1f}%")
            else:
                parts.append(f"battery: {v:.2f}V")

        if parts:
            self.get_logger().info(" | ".join(parts))


def main():
    rclpy.init()
    node = P500Ros2Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
