#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Host VLA Client
===============
在 VLA 主机端运行，封装与 Jetson 的 ZMQ 通信，对 VLA 模型开发者暴露
简洁的 Python API。

快速示例::

    from host_vla_client import RobotEnv
    import numpy as np

    env = RobotEnv(jetson_ip="192.168.x.x")
    obs = env.reset()

    for step in range(200):
        action = {
            "chassis"   : [0.1, 0.0, 0.0],          # vx=0.1 m/s
            "arm_delta" : np.zeros(6, dtype=float),   # 保持静止
            "gripper"   : 1.0,                        # 打开
        }
        obs, info = env.step(action)
        cam0_rgb = obs["cam0"]   # np.ndarray (H,W,3) uint8

    env.close()

依赖：
    pip install pyzmq msgpack msgpack-numpy numpy opencv-python
"""

from __future__ import annotations

import time
import logging
from typing import Any

import cv2
import numpy as np

try:
    import zmq
except ImportError:
    raise ImportError("请安装 pyzmq: pip install pyzmq")

try:
    import msgpack
    import msgpack_numpy as mnp
    mnp.patch()
except ImportError:
    raise ImportError("请安装 msgpack 和 msgpack-numpy: pip install msgpack msgpack-numpy")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("VLAClient")

# ARM_NUM_JOINTS 须与 jetson_robot_server.py 中的设置保持一致
ARM_NUM_JOINTS = 6


# ══════════════════════════════════════════════════════════════════════════════
#  数据类型定义（方便 IDE 自动补全）
# ══════════════════════════════════════════════════════════════════════════════
class Observation:
    """
    单步观测，由 :meth:`RobotEnv.step` 和 :meth:`RobotEnv.reset` 返回。

    属性
    ----
    joint_pos   : np.ndarray  形状 (ARM_NUM_JOINTS,)，单位 rad
    chassis_vel : np.ndarray  形状 (3,)，[vx, vy, wz] m/s / rad/s
    cam0        : np.ndarray  形状 (H, W, 3)，uint8 RGB
    cam1        : np.ndarray  形状 (H, W, 3)，uint8 RGB
    timestamp   : float       Unix 时间戳（Jetson 侧）
    """

    def __init__(self, raw: dict):
        self.joint_pos   = np.array(raw["joint_pos"],   dtype=np.float32)
        self.chassis_vel = np.array(raw["chassis_vel"], dtype=np.float32)
        self.cam0        = _decode_jpeg(raw.get("cam0", b""))
        self.cam1        = _decode_jpeg(raw.get("cam1", b""))
        self.timestamp   = float(raw.get("timestamp", 0.0))

    def as_dict(self) -> dict:
        """转为 dict，方便直接传给 VLA 模型的 forward()。"""
        return {
            "joint_pos"  : self.joint_pos,
            "chassis_vel": self.chassis_vel,
            "cam0"       : self.cam0,
            "cam1"       : self.cam1,
            "timestamp"  : self.timestamp,
        }

    def __repr__(self):
        def _arr(a):
            return np.round(a, 4).tolist()
        return (
            f"Observation(\n"
            f"  joint_pos   = {_arr(self.joint_pos)},\n"
            f"  chassis_vel = {_arr(self.chassis_vel)},\n"
            f"  cam0.shape  = {self.cam0.shape},\n"
            f"  cam1.shape  = {self.cam1.shape},\n"
            f"  timestamp   = {self.timestamp:.3f}\n"
            f")"
        )


def _decode_jpeg(data: bytes) -> np.ndarray:
    """将 JPEG bytes 解码为 (H,W,3) uint8 RGB 数组；失败时返回空黑图。"""
    if not data:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    buf = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


# ══════════════════════════════════════════════════════════════════════════════
#  主接口
# ══════════════════════════════════════════════════════════════════════════════
class RobotEnv:
    """
    面向 VLA 模型开发者的机器人环境接口。

    参数
    ----
    jetson_ip : str
        Jetson 的 IP 地址（与 VLA 主机在同一局域网）。
    port : int
        ZMQ 端口，须与 jetson_robot_server.py 中的 ``--zmq-port`` 一致。
    timeout_ms : int
        单步通信超时（毫秒）。
    """

    def __init__(
        self,
        jetson_ip: str = "192.168.1.200",
        port: int = 7788,
        timeout_ms: int = 5000,
    ):
        self._addr = f"tcp://{jetson_ip}:{port}"
        self._timeout = timeout_ms
        self._ctx    = zmq.Context()
        self._sock   = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._sock.connect(self._addr)
        log.info(f"[VLAClient] 已连接 {self._addr}")

    # ── 核心接口 ──────────────────────────────────────────────────────────────

    def step(self, action: dict) -> tuple[Observation, dict]:
        """
        发送一步动作，返回下一步观测。

        参数
        ----
        action : dict
            必须包含以下键：

            - ``"chassis"``  : ``list[float]`` 或 ``np.ndarray``，形状 (3,)。
              分别为 ``[vx(m/s), vy(m/s), wz(rad/s)]``。
              P500 为差速底盘时 vy 填 0.0。

            - ``"arm_delta"`` : ``list[float]`` 或 ``np.ndarray``，
              形状 ``(ARM_NUM_JOINTS,)``，单位 rad，
              代表各关节相对当前位置的**角度增量**。

            - ``"gripper"``  : ``float``，范围 ``[0.0, 1.0]``。
              ``0.0`` 为完全关闭，``1.0`` 为完全打开。

            可选键：

            - ``"arm_velocity"`` : ``int``，机械臂运动速度 0~1000（默认 300）。

        返回
        ----
        obs  : :class:`Observation`
        info : dict，包含 ``latency_ms``（往返延迟）。

        示例
        ----
        >>> obs, info = env.step({
        ...     "chassis"   : [0.1, 0.0, 0.0],
        ...     "arm_delta" : [0, 0, 0, 0.05, 0, 0],
        ...     "gripper"   : 0.0,
        ... })
        >>> print(obs.joint_pos)
        """
        packed = _pack_action(action)
        t0 = time.perf_counter()
        try:
            self._sock.send(packed)
            raw = self._sock.recv()
        except zmq.Again:
            raise TimeoutError(
                f"[VLAClient] 超时 {self._timeout} ms，请检查 Jetson 是否在线"
            )
        latency = (time.perf_counter() - t0) * 1000
        obs_raw = msgpack.unpackb(raw, raw=False)
        return Observation(obs_raw), {"latency_ms": latency}

    def reset(self) -> Observation:
        """
        发送零动作，返回初始观测（用于 episode 第一帧）。
        """
        obs, _ = self.step({
            "chassis"   : [0.0, 0.0, 0.0],
            "arm_delta" : [0.0] * ARM_NUM_JOINTS,
            "gripper"   : 0.5,
        })
        return obs

    def send_chassis(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        """
        仅发送底盘速度，机械臂和夹爪保持不动。
        适用于纯底盘导航场景。
        """
        self.step({
            "chassis"   : [vx, vy, wz],
            "arm_delta" : [0.0] * ARM_NUM_JOINTS,
            "gripper"   : 0.5,
        })

    def send_arm_delta(self, delta: list | np.ndarray,
                       velocity: int = 300) -> Observation:
        """
        仅发送机械臂关节增量，底盘停止。

        参数
        ----
        delta    : 各关节增量（rad），长度须等于 ARM_NUM_JOINTS。
        velocity : 运动速度 0~1000。
        """
        obs, _ = self.step({
            "chassis"     : [0.0, 0.0, 0.0],
            "arm_delta"   : list(delta),
            "gripper"     : 0.5,
            "arm_velocity": velocity,
        })
        return obs

    def set_gripper(self, value: float) -> Observation:
        """
        仅控制夹爪，其他保持不动。

        参数
        ----
        value : 0.0 = 完全关闭, 1.0 = 完全打开。
        """
        obs, _ = self.step({
            "chassis"   : [0.0, 0.0, 0.0],
            "arm_delta" : [0.0] * ARM_NUM_JOINTS,
            "gripper"   : value,
        })
        return obs

    def stop(self):
        """立即停止底盘和机械臂（发送零动作）。"""
        self.reset()

    def close(self):
        """断开连接，释放 ZMQ 资源。"""
        try:
            self.stop()
        except Exception:
            pass
        self._sock.close()
        self._ctx.term()
        log.info("[VLAClient] 已断开连接。")

    # ── 上下文管理器 ──────────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


# ══════════════════════════════════════════════════════════════════════════════
#  内部工具
# ══════════════════════════════════════════════════════════════════════════════
def _pack_action(action: dict) -> bytes:
    """将 action dict 序列化为 msgpack bytes，numpy 数组自动转 list。"""
    def _to_list(v):
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, (list, tuple)):
            return [float(x) for x in v]
        return v

    clean = {
        "chassis"     : _to_list(action.get("chassis",   [0.0, 0.0, 0.0])),
        "arm_delta"   : _to_list(action.get("arm_delta", [0.0] * ARM_NUM_JOINTS)),
        "gripper"     : float(action.get("gripper", 0.5)),
        "arm_velocity": int(action.get("arm_velocity", 300)),
    }
    return msgpack.packb(clean, use_bin_type=True)


# ══════════════════════════════════════════════════════════════════════════════
#  命令行快速测试
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="VLA Client 快速连通性测试")
    parser.add_argument("--jetson-ip", default="192.168.1.200")
    parser.add_argument("--port",      type=int, default=7788)
    parser.add_argument("--steps",     type=int, default=10)
    args = parser.parse_args()

    with RobotEnv(jetson_ip=args.jetson_ip, port=args.port) as env:
        print("[Test] 发送 reset()…")
        obs = env.reset()
        print(obs)

        print(f"\n[Test] 运行 {args.steps} 步零动作…")
        for i in range(args.steps):
            obs, info = env.step({
                "chassis"   : [0.0, 0.0, 0.0],
                "arm_delta" : [0.0] * ARM_NUM_JOINTS,
                "gripper"   : 0.5,
            })
            print(
                f"  step {i+1:3d} | latency={info['latency_ms']:.1f}ms"
                f" | joint_pos={np.round(obs.joint_pos, 3).tolist()}"
            )
            time.sleep(0.1)

    print("[Test] 完成。")
