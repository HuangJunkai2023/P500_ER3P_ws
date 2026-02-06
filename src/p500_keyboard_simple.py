#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P500 最简实时键盘控制: 启动清错+解除暂停, WASD 按下动松开停"""

import os
import sys
import time
import json
import threading
import termios
import tty
import select

import websocket

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, 'p500'))
import robot_api as chassis_api


class P500SimpleTeleop:
    def __init__(self, ip="192.168.1.100", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.ws = None
        self.ws_connected = False
        self.ws_thread = None
        self.last_key_time = {}
        self.key_hold_timeout = 0.2

        self._connect()

    def _connect(self):
        print(f"[P500] 连接到底盘 {self.ip}...")
        chassis_api.setHost(self.ip)
        res = chassis_api.login(self.username, self.password)
        if not res:
            raise RuntimeError(f"登录失败: {getattr(res, 'msg', '')}")
        print("[P500] 登录成功")

        # 先尝试退出充电/停止导航，避免进入错误态
        try:
            res = chassis_api.charge(False)
            print(f"[P500] 退出充电: {bool(res)} {getattr(res, 'msg', '')}")
        except Exception as e:
            print(f"[P500] 退出充电异常: {e}")
        try:
            res = chassis_api.stop()
            print(f"[P500] 停止导航/建图: {bool(res)} {getattr(res, 'msg', '')}")
        except Exception as e:
            print(f"[P500] 停止导航异常: {e}")

        # 先启动建图进入可控状态（无地图也能进入）
        try:
            res = chassis_api.mapping()
            print(f"[P500] 启动建图: {bool(res)} {getattr(res, 'msg', '')}")
        except Exception as e:
            print(f"[P500] 启动建图异常: {e}")

        # 建图启动需要时间，轮询状态后再清错/解除暂停
        nav_ready = False
        for _ in range(30):
            try:
                st = chassis_api.status()
                if st and getattr(st, "data", None) == 1:
                    nav_ready = True
                    break
            except Exception:
                pass
            time.sleep(1.0)

        # 打印一次状态，便于确认模式/错误
        try:
            rs = chassis_api.robotStatus()
            if rs and rs.data:
                data = rs.data
                print(f"[P500] 状态: state={data.get('state')}, chassis_mode={data.get('chassis_mode')}, err={data.get('err')}, msg={data.get('msg')}")
        except Exception as e:
            print(f"[P500] 获取状态异常: {e}")

        if nav_ready:
            try:
                res = chassis_api.clear()
                print(f"[P500] 清除错误: {bool(res)} {getattr(res, 'msg', '')}")
            except Exception as e:
                print(f"[P500] 清除错误异常: {e}")

            try:
                res = chassis_api.pause(False)
                print(f"[P500] 解除暂停: {bool(res)} {getattr(res, 'msg', '')}")
            except Exception as e:
                print(f"[P500] 解除暂停异常: {e}")
        else:
            print("[P500] 建图状态未就绪，跳过清错/解除暂停")

    def start_ws(self):
        if self.ws_connected:
            return

        ws_url = f"ws://{self.ip}:8080/api/ws?token={chassis_api.sess.headers.get('Authorization', '')}"

        def on_open(_ws):
            self.ws_connected = True
            print("[P500] WebSocket 连接已建立")

        def on_close(_ws, code, msg):
            self.ws_connected = False
            print(f"[P500] WebSocket 连接已关闭: {code} - {msg}")

        def on_error(_ws, err):
            print(f"[P500] WebSocket 错误: {err}")

        self.ws = websocket.WebSocketApp(ws_url, on_open=on_open, on_close=on_close, on_error=on_error)
        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()

    def send_cmd(self, vx, omega):
        if not self.ws_connected:
            self.start_ws()
            time.sleep(0.05)
        if not self.ws_connected or self.ws is None:
            return
        msg = {"type": 0, "data": {"linear": float(vx), "angular": float(omega)}}
        try:
            self.ws.send(json.dumps(msg))
        except Exception as e:
            print(f"[P500] 发送失败: {e}")

    def stop(self):
        self.send_cmd(0.0, 0.0)
        if self.ws:
            self.ws.close()


def realtime_keyboard_control():
    print("\n" + "=" * 60)
    print("P500 最简实时键盘控制")
    print("=" * 60)
    print("W 前进 | S 后退 | A 左转 | D 右转 | Q 退出")
    print("按下移动，松开停止")
    print("=" * 60 + "\n")

    controller = P500SimpleTeleop()
    controller.start_ws()
    time.sleep(0.5)

    linear_vel = 0.3
    angular_vel = 0.6

    if not sys.stdin.isatty():
        print("[P500] 当前终端不是 TTY，无法实时按键控制。请用真实终端运行。")
        print("示例：conda activate xmate && python p500_keyboard_simple.py")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    print("\n退出程序...")
                    break
                if key in ('w', 'a', 's', 'd'):
                    controller.last_key_time[key] = time.time()

            now = time.time()
            active = {k for k, t in controller.last_key_time.items() if now - t <= controller.key_hold_timeout}

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

            controller.send_cmd(vx, omega)
            time.sleep(0.05)  # 20Hz
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        controller.stop()
        print("[P500] 已停止")


if __name__ == '__main__':
    realtime_keyboard_control()
