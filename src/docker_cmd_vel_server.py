#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive UDP velocity commands and publish to /cmd_vel (ROS1)."""

import select
import signal
import socket
import threading
import time
import sys
import argparse

import rospy
from geometry_msgs.msg import Twist


def parse_payload(data):
    try:
        text = data.decode("utf-8").strip()
        if not text:
            return None
        parts = text.split()
        if len(parts) != 2:
            return None
        linear_x = float(parts[0])
        angular_z = float(parts[1])
        return linear_x, angular_z
    except Exception:
        return None


def parse_ros_args(argv):
    params = {}
    for item in argv:
        if item.startswith("_") and ":=" in item:
            key, val = item[1:].split(":=", 1)
            params[key] = val
    return params


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--udp-port", type=int, default=15000)
    parser.add_argument("--stop-timeout", type=float, default=0.2)
    parser.add_argument("--publish", action="store_true", default=False)
    parser.add_argument("--no-ros", action="store_true", default=False)
    args, unknown = parser.parse_known_args()

    ros_params = parse_ros_args(unknown)

    udp_port = int(ros_params.get("udp_port", args.udp_port))
    stop_timeout = float(ros_params.get("stop_timeout", args.stop_timeout))

    publish_cmd_vel = args.publish
    if "publish" in ros_params:
        publish_cmd_vel = ros_params.get("publish", "true").lower() == "true"

    use_ros = publish_cmd_vel and not args.no_ros

    if use_ros:
        rospy.init_node("udp_cmd_vel_server", anonymous=True, disable_signals=True)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    else:
        pub = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", udp_port))
    sock.settimeout(0.1)

    running = True

    def on_shutdown(reason="shutdown"):
        nonlocal running
        running = False
        try:
            rospy.signal_shutdown(reason)
        except Exception:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def handle_signal(_signum, _frame):
        on_shutdown("signal")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rospy.on_shutdown(on_shutdown)

    last_cmd_time = 0.0
    last_twist = Twist()

    print(f"[UDP] Listening on 0.0.0.0:{udp_port} (publish={publish_cmd_vel})", flush=True)

    def receiver():
        nonlocal last_cmd_time, last_twist
        while running and (not use_ros or not rospy.is_shutdown()):
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            cmd = parse_payload(data)
            if cmd is None:
                continue
            linear_x, angular_z = cmd
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            print(
                f"[{ts}] [UDP] {addr[0]}:{addr[1]} -> linear_x={linear_x:.3f}, angular_z={angular_z:.3f}",
                flush=True,
            )
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            last_twist = twist
            last_cmd_time = time.time()
            if publish_cmd_vel and pub is not None:
                pub.publish(twist)

    thread = threading.Thread(target=receiver, daemon=True)
    thread.start()

    try:
        while running and (not use_ros or not rospy.is_shutdown()):
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.read(1).lower() == "q":
                    on_shutdown("quit")
                    break
            if publish_cmd_vel and pub is not None:
                if time.time() - last_cmd_time > stop_timeout:
                    pub.publish(Twist())
                else:
                    pub.publish(last_twist)
            time.sleep(0.05)
    except KeyboardInterrupt:
        on_shutdown("keyboard")


if __name__ == "__main__":
    main()
