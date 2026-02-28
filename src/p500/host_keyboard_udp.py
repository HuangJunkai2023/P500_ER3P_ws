#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard teleop that sends UDP velocity commands to Docker."""

import argparse
import socket
import sys
import termios
import tty
import select
import time


KEY_BINDINGS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
}


def get_key(timeout_sec=0.01):
    if select.select([sys.stdin], [], [], timeout_sec)[0]:
        return sys.stdin.read(1)
    return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1", help="Docker host IP")
    parser.add_argument("--port", type=int, default=15000, help="UDP port")
    parser.add_argument("--linear", type=float, default=0.15, help="Linear x m/s")
    parser.add_argument("--angular", type=float, default=0.2, help="Angular z rad/s")
    parser.add_argument("--rate", type=float, default=30.0, help="Send rate Hz")
    parser.add_argument("--stop-timeout", type=float, default=0.05, help="Stop after idle seconds")
    parser.add_argument("--key-timeout", type=float, default=0.01, help="Key poll timeout seconds")
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print("[P500] This terminal is not a TTY. Run in a real terminal.")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / max(args.rate, 1e-3)

    print("\nP500 UDP keyboard teleop")
    print("W/S: forward/back, A/D: turn left/right, Q: quit")
    print(f"Linear={args.linear} m/s, Angular={args.angular} rad/s")
    print(f"Target={args.host}:{args.port}, Rate={args.rate} Hz")

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    last_cmd_time = 0.0
    last_payload = b"0.0 0.0"

    try:
        next_send = time.monotonic()
        while True:
            key = get_key(timeout_sec=args.key_timeout)
            if key:
                key = key.lower()
                if key == "q":
                    break
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    linear_x = lin * args.linear
                    angular_z = ang * args.angular
                    last_payload = f"{linear_x} {angular_z}".encode("utf-8")
                    last_cmd_time = time.time()

            if time.time() - last_cmd_time > args.stop_timeout:
                last_payload = b"0.0 0.0"

            now = time.monotonic()
            if now >= next_send:
                sock.sendto(last_payload, (args.host, args.port))
                next_send = now + interval
    finally:
        sock.sendto(b"0.0 0.0", (args.host, args.port))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
