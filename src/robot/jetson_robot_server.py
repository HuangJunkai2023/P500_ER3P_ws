#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Robot Server
===================
在 Jetson 端运行，统一管理所有硬件，并向 VLA 主机提供 ZMQ 接口。

动作输入（来自主机）:
  chassis   : [vx, vy, wz]    m/s, m/s, rad/s
  arm_delta : [dj0..dj5]      rad 增量（关节数由 ARM_NUM_JOINTS 控制，默认 6）
  gripper   : float            0.0=关闭, 1.0=全开

观测输出（返回主机）:
  joint_pos   : [j0..j5]      rad
  chassis_vel : [vx, vy, wz]
  cam0        : bytes          JPEG 压缩 RGB
  cam1        : bytes          JPEG 压缩 RGB
  timestamp   : float          UTC Unix 时间戳

依赖：
  pip install pyzmq msgpack numpy opencv-python
  xCoreSDK: src/xcoresdk_python-v0.6.0
  Jodell:   src/jodell  （夹爪，可选）
"""

import sys
import os
import time
import socket
import threading
import platform
import argparse
import logging
import struct
from datetime import timedelta

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

# ──────────────────────────── 机械臂 SDK ────────────────────────────────────
SDK_PATH = os.path.join(os.path.dirname(__file__),
                        "../../xcoresdk_python-v0.6.0/example")
sys.path.insert(0, os.path.normpath(SDK_PATH))

try:
    import setup_path  # noqa: 自动将 Release/ 加入 sys.path
    if platform.system() == "Linux":
        from Release.linux import xCoreSDK_python
        from Release.linux.xCoreSDK_python import RtSupportedFields
    else:
        from Release.windows import xCoreSDK_python
        from Release.windows.xCoreSDK_python import RtSupportedFields
    ARM_SDK_AVAILABLE = True
except Exception as e:
    logging.warning(f"[ARM] xCoreSDK 加载失败，机械臂接口将以 stub 模式运行: {e}")
    ARM_SDK_AVAILABLE = False

# ──────────────────────────── 夹爪 SDK（可选）──────────────────────────────
JODELL_PATH = os.path.join(os.path.dirname(__file__), "../../jodell")
sys.path.insert(0, os.path.normpath(JODELL_PATH))
try:
    from er3_io_ctrl import ER3IOCtrl  # type: ignore
    GRIPPER_SDK_AVAILABLE = True
except Exception:
    GRIPPER_SDK_AVAILABLE = False

# ──────────────────────────── 常量 ─────────────────────────────────────────
ARM_NUM_JOINTS = 6        # ER3P 为 6 轴；若为 7 轴变体请改为 7
JPEG_QUALITY   = 80       # 图像压缩质量
CAM0_INDEX     = 0        # /dev/video0
CAM1_INDEX     = 2        # /dev/video2（常见 Jetson CSI 索引）
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
CAM_FPS        = 30
ZMQ_PORT       = 7788

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("JetsonServer")


# ══════════════════════════════════════════════════════════════════════════════
#  底盘控制（UDP → P500 Docker 容器）
# ══════════════════════════════════════════════════════════════════════════════
class ChassisController:
    """
    通过 UDP 向 P500 底盘的 Docker 容器发送速度指令。
    格式: "<vx> <vy> <wz>\\n"（UTF-8 文本，单位 m/s / rad/s）
    """

    def __init__(self, p500_ip="192.168.1.100", udp_port=15000):
        self.addr = (p500_ip, udp_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        # 状态缓存（由外部周期性更新，此处仅保留最后下发值）
        self._vx = self._vy = self._wz = 0.0
        log.info(f"[Chassis] UDP → {p500_ip}:{udp_port}")

    def set_velocity(self, vx: float, vy: float, wz: float):
        with self._lock:
            self._vx, self._vy, self._wz = vx, vy, wz
        payload = f"{vx:.4f} {vy:.4f} {wz:.4f}".encode()
        try:
            self.sock.sendto(payload, self.addr)
        except OSError as e:
            log.warning(f"[Chassis] UDP 发送失败: {e}")

    def stop(self):
        self.set_velocity(0.0, 0.0, 0.0)

    def get_velocity(self):
        """返回最后下发的速度（Jetson 侧无底盘速度反馈时用此近似）"""
        with self._lock:
            return [self._vx, self._vy, self._wz]

    def close(self):
        self.stop()
        self.sock.close()


# ══════════════════════════════════════════════════════════════════════════════
#  机械臂控制（珞石 ER3P · xCoreSDK）
# ══════════════════════════════════════════════════════════════════════════════
class ArmController:
    """
    基于 xCoreSDK 的机械臂控制器。
    - 订阅关节角流式数据（高频）
    - 支持以关节角增量执行 MoveAbsJ
    """

    # ER3P 各关节软限位（rad），按需修改
    JOINT_LIMITS = [
        (-2.967, 2.967),
        (-2.094, 2.094),
        (-2.967, 2.967),
        (-2.094, 2.094),
        (-2.967, 2.967),
        (-3.142, 3.142),
    ]

    def __init__(self, arm_ip="192.168.21.10", local_ip="192.168.21.1",
                 state_interval_ms=10):
        self._arm_ip = arm_ip
        self._local_ip = local_ip
        self._interval_ms = state_interval_ms
        self._ec: dict = {}
        self._robot = None
        self._joint_buf = None
        self._state_lock = threading.Lock()
        self._joint_pos = [0.0] * ARM_NUM_JOINTS
        self._stream_thread = None
        self._streaming = False

        if ARM_SDK_AVAILABLE:
            self._init_sdk()

    def _init_sdk(self):
        try:
            self._robot = xCoreSDK_python.xMateRobot(self._arm_ip, self._local_ip)
            self._robot.connectToRobot(self._ec)
            self._robot.setOperateMode(xCoreSDK_python.OperateMode.automatic, self._ec)
            self._robot.setPowerState(True, self._ec)
            self._robot.setMotionControlMode(
                xCoreSDK_python.MotionControlMode.NrtCommandMode, self._ec)
            log.info(f"[Arm] 已连接 ER3P @ {self._arm_ip}")

            # 启动关节角流式订阅
            self._joint_buf = xCoreSDK_python.PyTypeVectorDouble()
            self._robot.startReceiveRobotState(
                timedelta(milliseconds=self._interval_ms),
                [RtSupportedFields.jointPos_m],
            )
            # 清空历史帧
            while self._robot.updateRobotState(timedelta(milliseconds=0)):
                pass

            self._streaming = True
            self._stream_thread = threading.Thread(
                target=self._stream_loop, daemon=True)
            self._stream_thread.start()
        except Exception as e:
            log.error(f"[Arm] SDK 初始化失败: {e}")
            self._robot = None

    def _stream_loop(self):
        """后台线程持续更新关节角缓存。"""
        timeout = timedelta(milliseconds=self._interval_ms * 2)
        while self._streaming and self._robot is not None:
            try:
                self._robot.updateRobotState(timeout)
                self._robot.getStateData(
                    RtSupportedFields.jointPos_m,
                    self._joint_buf,
                    ARM_NUM_JOINTS,
                )
                pos = list(self._joint_buf.content())
                with self._state_lock:
                    self._joint_pos = pos
            except Exception as e:
                log.warning(f"[Arm] 流式读取异常: {e}")
                time.sleep(0.01)

    def get_joint_pos(self) -> list:
        """返回最新关节角（rad），线程安全。"""
        if not ARM_SDK_AVAILABLE or self._robot is None:
            return self._joint_pos  # stub: 返回 0
        with self._state_lock:
            return list(self._joint_pos)

    def move_delta(self, delta: list, velocity: int = 300):
        """
        以关节角增量执行运动。
        velocity: 运动速度（0-1000，建议 VLA 推理用 200-400）
        """
        if not ARM_SDK_AVAILABLE or self._robot is None:
            with self._state_lock:
                for i in range(min(len(delta), ARM_NUM_JOINTS)):
                    self._joint_pos[i] += delta[i]
            return

        current = self.get_joint_pos()
        target = []
        for i in range(ARM_NUM_JOINTS):
            d = delta[i] if i < len(delta) else 0.0
            lo, hi = self.JOINT_LIMITS[i] if i < len(self.JOINT_LIMITS) else (-6.28, 6.28)
            target.append(float(np.clip(current[i] + d, lo, hi)))

        try:
            cmd = xCoreSDK_python.MoveAbsJCommand(target, velocity)
            id_str = xCoreSDK_python.PyString()
            self._robot.moveReset(self._ec)
            self._robot.moveAppend([cmd], id_str, self._ec)
            self._robot.moveStart(self._ec)
        except Exception as e:
            log.warning(f"[Arm] 运动指令失败: {e}")

    def close(self):
        self._streaming = False
        if self._robot is not None:
            try:
                self._robot.stopReceiveRobotState()
                self._robot.setPowerState(False, self._ec)
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════════
#  夹爪控制（Jodell）
# ══════════════════════════════════════════════════════════════════════════════
class GripperController:
    """
    夹爪控制器，基于 src/jodell/er3_io_ctrl.py。
    gripper_val: 0.0 = 完全关闭, 1.0 = 完全打开（线性映射到夹爪行程）
    """

    MAX_POSITION = 1000  # 夹爪最大位置（按 Jodell 量程调整）

    def __init__(self, port="/dev/ttyUSB0"):
        self._ctrl = None
        if GRIPPER_SDK_AVAILABLE:
            try:
                self._ctrl = ER3IOCtrl(port)
                log.info(f"[Gripper] Jodell 夹爪已连接 @ {port}")
            except Exception as e:
                log.warning(f"[Gripper] 初始化失败（stub 模式）: {e}")
        else:
            log.warning("[Gripper] Jodell SDK 未加载，以 stub 模式运行")

    def set(self, value: float):
        """value: 0.0（关）~ 1.0（开），线程安全调用。"""
        value = float(np.clip(value, 0.0, 1.0))
        if self._ctrl is None:
            log.debug(f"[Gripper] stub set({value:.2f})")
            return
        try:
            pos = int(value * self.MAX_POSITION)
            self._ctrl.set_position(pos)
        except Exception as e:
            log.warning(f"[Gripper] 控制失败: {e}")

    def close(self):
        if self._ctrl is not None:
            try:
                self._ctrl.close()
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════════
#  相机管理
# ══════════════════════════════════════════════════════════════════════════════
class CameraManager:
    """
    管理双路相机，后台线程持续抓帧并缓存 JPEG 数据。
    """

    def __init__(self, cam0_idx=CAM0_INDEX, cam1_idx=CAM1_INDEX,
                 width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS,
                 quality=JPEG_QUALITY):
        self._quality = quality
        self._lock = threading.Lock()
        self._frames = [None, None]   # JPEG bytes
        self._cams: list = []
        self._running = False

        for idx in (cam0_idx, cam1_idx):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                log.info(f"[Camera] /dev/video{idx} 已打开 {width}x{height}@{fps}")
            else:
                log.warning(f"[Camera] /dev/video{idx} 无法打开，将返回空帧")
                cap = None
            self._cams.append(cap)

    def start(self):
        self._running = True
        t = threading.Thread(target=self._capture_loop, daemon=True)
        t.start()

    def _capture_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self._quality]
        blank = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
        _, blank_jpg = cv2.imencode(".jpg", blank, encode_param)
        blank_bytes = blank_jpg.tobytes()

        while self._running:
            frames = []
            for cap in self._cams:
                if cap is not None and cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        _, buf = cv2.imencode(".jpg", frame, encode_param)
                        frames.append(buf.tobytes())
                        continue
                frames.append(blank_bytes)

            with self._lock:
                self._frames = frames
            time.sleep(1.0 / CAM_FPS)

    def get_frames(self):
        """返回 (cam0_jpg_bytes, cam1_jpg_bytes)。"""
        with self._lock:
            f = self._frames
        if len(f) < 2:
            empty = b""
            return empty, empty
        return f[0] or b"", f[1] or b""

    def close(self):
        self._running = False
        for cap in self._cams:
            if cap is not None:
                cap.release()


# ══════════════════════════════════════════════════════════════════════════════
#  ZMQ 服务端
# ══════════════════════════════════════════════════════════════════════════════
class JetsonRobotServer:
    """
    ZMQ REP 服务端——核心通信节点。

    协议（msgpack 编码的 dict）
    ──────────────────────────
    请求（Action）:
      {
        "chassis"   : [vx, vy, wz],        # float, m/s / rad/s
        "arm_delta" : [dj0, ..., dj5],     # float, rad 增量
        "gripper"   : float,               # 0.0~1.0
        "arm_velocity": int,               # 可选，默认 300
      }

    响应（Observation）:
      {
        "joint_pos"   : [j0, ..., j5],     # float, rad
        "chassis_vel" : [vx, vy, wz],      # float, m/s / rad/s
        "cam0"        : bytes,             # JPEG
        "cam1"        : bytes,             # JPEG
        "timestamp"   : float,             # Unix 时间戳
      }
    """

    def __init__(self, args):
        self.chassis  = ChassisController(args.p500_ip,  args.udp_port)
        self.arm      = ArmController(args.arm_ip, args.local_ip,
                                      args.arm_state_hz)
        self.gripper  = GripperController(args.gripper_port)
        self.cameras  = CameraManager(
            args.cam0, args.cam1,
            args.cam_width, args.cam_height, args.cam_fps, args.jpeg_quality,
        )
        self.cameras.start()

        self.ctx    = zmq.Context()
        self.socket = self.ctx.socket(zmq.REP)
        self.socket.bind(f"tcp://0.0.0.0:{args.zmq_port}")
        log.info(f"[Server] ZMQ REP 监听 0.0.0.0:{args.zmq_port}")

    def run(self):
        log.info("[Server] 就绪，等待 VLA 主机连接…")
        try:
            while True:
                raw = self.socket.recv()
                action = msgpack.unpackb(raw, raw=False)
                obs = self._step(action)
                self.socket.send(msgpack.packb(obs, use_bin_type=True))
        except KeyboardInterrupt:
            log.info("[Server] 收到中断，正在关闭…")
        finally:
            self._shutdown()

    def _step(self, action: dict) -> dict:
        # 1. 执行底盘指令
        chassis_cmd = action.get("chassis", [0.0, 0.0, 0.0])
        vx = float(chassis_cmd[0]) if len(chassis_cmd) > 0 else 0.0
        vy = float(chassis_cmd[1]) if len(chassis_cmd) > 1 else 0.0
        wz = float(chassis_cmd[2]) if len(chassis_cmd) > 2 else 0.0
        self.chassis.set_velocity(vx, vy, wz)

        # 2. 执行机械臂增量
        arm_delta = action.get("arm_delta", [0.0] * ARM_NUM_JOINTS)
        arm_vel   = int(action.get("arm_velocity", 300))
        self.arm.move_delta(arm_delta, velocity=arm_vel)

        # 3. 执行夹爪
        gripper_val = float(action.get("gripper", 0.5))
        self.gripper.set(gripper_val)

        # 4. 采集观测
        joint_pos   = self.arm.get_joint_pos()
        chassis_vel = self.chassis.get_velocity()
        cam0, cam1  = self.cameras.get_frames()

        return {
            "joint_pos"  : joint_pos,
            "chassis_vel": chassis_vel,
            "cam0"       : cam0,
            "cam1"       : cam1,
            "timestamp"  : time.time(),
        }

    def _shutdown(self):
        self.chassis.close()
        self.arm.close()
        self.gripper.close()
        self.cameras.close()
        self.socket.close()
        self.ctx.term()
        log.info("[Server] 已关闭。")


# ══════════════════════════════════════════════════════════════════════════════
#  入口
# ══════════════════════════════════════════════════════════════════════════════
def parse_args():
    p = argparse.ArgumentParser(description="Jetson Robot Server for VLA")
    # 网络
    p.add_argument("--zmq-port",    type=int,   default=7788)
    p.add_argument("--p500-ip",     default="192.168.1.100", help="P500 底盘 IP")
    p.add_argument("--udp-port",    type=int,   default=15000, help="P500 Docker UDP 端口")
    # 机械臂
    p.add_argument("--arm-ip",      default="192.168.21.10", help="ER3P 控制器 IP")
    p.add_argument("--local-ip",    default="192.168.21.1",  help="Jetson 本机 IP（臂侧网卡）")
    p.add_argument("--arm-state-hz",type=int,   default=100,  help="关节角流式更新频率 Hz")
    # 夹爪
    p.add_argument("--gripper-port",default="/dev/ttyUSB0",  help="夹爪串口")
    # 相机
    p.add_argument("--cam0",        type=int,   default=CAM0_INDEX)
    p.add_argument("--cam1",        type=int,   default=CAM1_INDEX)
    p.add_argument("--cam-width",   type=int,   default=CAM_WIDTH)
    p.add_argument("--cam-height",  type=int,   default=CAM_HEIGHT)
    p.add_argument("--cam-fps",     type=int,   default=CAM_FPS)
    p.add_argument("--jpeg-quality",type=int,   default=JPEG_QUALITY)
    return p.parse_args()


if __name__ == "__main__":
    server = JetsonRobotServer(parse_args())
    server.run()
