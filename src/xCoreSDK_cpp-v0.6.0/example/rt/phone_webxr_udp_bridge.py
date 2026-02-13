"""Phone WebXR -> UDP bridge for C++ teleop controller.

Run this script, then open the shown URL on iPhone Safari.
It forwards arm-mode phone pose to UDP CSV packets:
px,py,pz,qx,qy,qz,qw,enabled,mode
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import time
from pathlib import Path
from engineio.payload import Payload

DEFAULT_GRIPPER_PORT = "/dev/ttyUSB0"
DEFAULT_GRIPPER_ID = 9
DEFAULT_GRIPPER_BAUD = 115200
DEFAULT_GRIPPER_TORQUE = 80
DEFAULT_GRIPPER_SPEED = 200
DEFAULT_GRIPPER_STARTUP_LOC = 0
DEFAULT_GRIPPER_RESET_OPEN_LOC = 0
DEFAULT_GRIPPER_STARTUP_WAIT_S = 1.5
DEFAULT_GRIPPER_STARTUP_RETRY = 2
DEFAULT_GRIPPER_DELTA_GAIN = 1.5  # >1.0 means higher sensitivity (less finger travel)

# Safari may batch many polling packets when network jitters; raise decode cap.
Payload.max_decode_packets = 256


class BridgeServer:
    def __init__(
        self,
        web_root: str,
        web_port: int,
        udp_host: str,
        udp_port: int,
        cert_file: str,
        key_file: str,
    ):
        from flask import Flask, render_template
        from flask_socketio import SocketIO, emit

        self.udp_addr = (udp_host, udp_port)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.use_ssl = True
        self.cert_file = cert_file
        self.key_file = key_file
        self.rx_count = 0
        self.tx_count = 0
        self.last_mode = "none"
        self._last_stat_ts = time.time()
        self.gripper = EpgGripperController(
            port=DEFAULT_GRIPPER_PORT,
            slave_id=DEFAULT_GRIPPER_ID,
            baud=DEFAULT_GRIPPER_BAUD,
            torque=DEFAULT_GRIPPER_TORQUE,
            speed=DEFAULT_GRIPPER_SPEED,
        )

        tpl = os.path.join(web_root, "templates")
        sta = os.path.join(web_root, "static")
        self.app = Flask(__name__, template_folder=tpl, static_folder=sta)
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode="threading",
            allow_upgrades=False,
            logger=False,
            engineio_logger=False,
        )
        self.web_port = web_port

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.socketio.on("message")
        def on_msg(data):
            self.rx_count += 1
            ts = data.get("timestamp")
            if ts is not None:
                emit("echo", ts)

            state_update = data.get("state_update", "")
            reset_env = 1 if state_update == "reset_env" else 0
            gripper_delta = data.get("gripper_delta")
            if gripper_delta is not None:
                try:
                    gripper_delta = float(gripper_delta)
                except Exception:
                    gripper_delta = None

            # enabled = 1 only while touch teleop is active
            mode = 1 if data.get("teleop_mode") == "arm" else 0
            enabled = 1 if "teleop_mode" in data and mode == 1 else 0
            if enabled == 1:
                self.last_mode = "arm(enabled)"
            elif data.get("teleop_mode") == "base":
                self.last_mode = "base"
            elif reset_env == 1:
                self.last_mode = "reset_env"
            else:
                self.last_mode = "none"

            pos = data.get("position", {})
            ori = data.get("orientation", {})

            px = float(pos.get("x", 0.0))
            py = float(pos.get("y", 0.0))
            pz = float(pos.get("z", 0.0))
            qx = float(ori.get("x", 0.0))
            qy = float(ori.get("y", 0.0))
            qz = float(ori.get("z", 0.0))
            qw = float(ori.get("w", 1.0))
            gd = 0.0 if gripper_delta is None else gripper_delta

            pkt = f"{px},{py},{pz},{qx},{qy},{qz},{qw},{enabled},{mode},{reset_env},{gd}\n".encode("utf-8")
            self.udp_sock.sendto(pkt, self.udp_addr)
            self.tx_count += 1
            if reset_env == 1:
                self.gripper.open_for_reset()
            self.gripper.process(enabled == 1 and mode == 1, gripper_delta)

            now = time.time()
            if now - self._last_stat_ts >= 1.0:
                print(f"[bridge] rx={self.rx_count} tx={self.tx_count} mode={self.last_mode}")
                self._last_stat_ts = now

        logging.getLogger("werkzeug").setLevel(logging.WARNING)

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(("8.8.8.8", 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = "127.0.0.1"
        finally:
            s.close()

        protocol = "https" if self.use_ssl else "http"
        print(f"Phone web app: {protocol}://{ip}:{self.web_port}")
        print(f"Forwarding UDP to {self.udp_addr[0]}:{self.udp_addr[1]}")
        if self.use_ssl:
            if not (os.path.exists(self.cert_file) and os.path.exists(self.key_file)):
                raise FileNotFoundError(
                    "SSL enabled but cert/key not found.\n"
                    f"cert: {self.cert_file}\n"
                    f"key : {self.key_file}\n"
                    "Generate with:\n"
                    "openssl req -x509 -newkey rsa:4096 -nodes "
                    f"-out {self.cert_file} -keyout {self.key_file} -days 365"
                )
            self.socketio.run(
                self.app,
                host="0.0.0.0",
                port=self.web_port,
                ssl_context=(self.cert_file, self.key_file),
            )
        else:
            self.socketio.run(self.app, host="0.0.0.0", port=self.web_port)


class EpgGripperController:
    """滑动控制Jodell EPG夹爪（参考 tidybot2 gripper_delta 语义）。"""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        slave_id: int = 9,
        baud: int = 115200,
        torque: int = 80,
        speed: int = 200,
        min_cmd_interval_s: float = 0.08,
        min_step_norm: float = 0.01,
    ):
        self.port = port
        self.slave_id = slave_id
        self.baud = baud
        self.torque = int(max(0, min(255, torque)))
        self.speed = int(max(1, min(255, speed)))
        self.min_cmd_interval_s = min_cmd_interval_s
        self.min_step_norm = min_step_norm
        self.claw = None
        self.ref_norm = 0.0
        self.target_norm = 0.0
        self.last_sent_norm = None
        self.last_cmd_ts = 0.0
        self.active = False
        self._init_ok = self._init_device()

    def _init_device(self) -> bool:
        try:
            from jodellSdk.jodellSdkDemo import EpgClawControl  # type: ignore
        except Exception:
            # Fallback: load SDK wheel from jodell python directory in this workspace.
            wheel = (
                Path(__file__).resolve().parents[3]
                / "jodell"
                / "python"
                / "JodellTool-0.1.5-py3-none-any.whl"
            )
            if not wheel.exists():
                print("[gripper] SDK not found, disable gripper control")
                return False
            import sys

            sys.path.insert(0, str(wheel))
            try:
                from jodellSdk.jodellSdkDemo import EpgClawControl  # type: ignore
            except Exception as e:
                print(f"[gripper] SDK import failed: {e}")
                return False

        try:
            self.claw = EpgClawControl()
            if self.claw.serialOperation(self.port, self.baud, True) != 1:
                print(f"[gripper] serial open failed: {self.port}@{self.baud}")
                return False
            if self.claw.enableClamp(self.slave_id, True) != 1:
                print(f"[gripper] enable clamp failed: id={self.slave_id}")
                return False
            time.sleep(0.3)
            # User requested startup action: move clamp to location 255.
            loc = self.claw.getClampCurrentLocation(self.slave_id)
            loc_i = self._loc_int(loc)
            for i in range(DEFAULT_GRIPPER_STARTUP_RETRY):
                ret = self.claw.runWithParam(
                    self.slave_id,
                    DEFAULT_GRIPPER_STARTUP_LOC,
                    self.speed,
                    self.torque,
                )
                time.sleep(DEFAULT_GRIPPER_STARTUP_WAIT_S)
                loc = self.claw.getClampCurrentLocation(self.slave_id)
                loc_i = self._loc_int(loc)
                print(f"[gripper] startup try={i+1}, ret={ret}, loc={loc_i}")
                if ret == 1 and abs(loc_i - DEFAULT_GRIPPER_STARTUP_LOC) <= 5:
                    break
            self.ref_norm = self._norm_from_loc(loc)
            self.target_norm = self.ref_norm
            self.last_sent_norm = self.ref_norm
            print(
                f"[gripper] enabled: port={self.port}, id={self.slave_id}, "
                f"startup_loc={DEFAULT_GRIPPER_STARTUP_LOC}, pos={self.ref_norm:.3f}"
            )
            return True
        except Exception as e:
            print(f"[gripper] init failed: {e}")
            return False

    @staticmethod
    def _norm_from_loc(loc) -> float:
        try:
            v = float(loc)
        except Exception:
            v = 0.0
        return max(0.0, min(1.0, v / 255.0))

    @staticmethod
    def _loc_int(loc) -> int:
        try:
            v = int(round(float(loc)))
        except Exception:
            v = 0
        return max(0, min(255, v))

    @staticmethod
    def _loc_from_norm(norm: float) -> int:
        v = int(round(max(0.0, min(1.0, norm)) * 255.0))
        return max(0, min(255, v))

    def process(self, arm_enabled: bool, gripper_delta):
        if not self._init_ok:
            return

        if not arm_enabled:
            self.active = False
            return

        # Rising edge: capture current clamp position as episode reference.
        if not self.active:
            self.active = True
            try:
                self.ref_norm = self._norm_from_loc(self.claw.getClampCurrentLocation(self.slave_id))
                self.target_norm = self.ref_norm
                self.last_sent_norm = self.ref_norm
            except Exception:
                pass

        if gripper_delta is None:
            return

        d = float(max(-1.0, min(1.0, gripper_delta * DEFAULT_GRIPPER_DELTA_GAIN)))
        self.target_norm = max(0.0, min(1.0, self.ref_norm + d))

        now = time.time()
        if self.last_sent_norm is not None and abs(self.target_norm - self.last_sent_norm) < self.min_step_norm:
            return
        if now - self.last_cmd_ts < self.min_cmd_interval_s:
            return

        loc = self._loc_from_norm(self.target_norm)
        try:
            ret = self.claw.runWithParam(self.slave_id, loc, self.speed, self.torque)
            if ret == 1:
                self.last_sent_norm = self.target_norm
                self.last_cmd_ts = now
            else:
                print(f"[gripper] runWithParam failed: {ret}")
        except Exception as e:
            print(f"[gripper] command error: {e}")

    def open_for_reset(self):
        if not self._init_ok:
            return
        try:
            ret = self.claw.runWithParam(
                self.slave_id,
                DEFAULT_GRIPPER_RESET_OPEN_LOC,
                self.speed,
                self.torque,
            )
            if ret != 1:
                print(f"[gripper] reset open failed: {ret}")
                return
            time.sleep(0.2)
            loc = self._loc_int(self.claw.getClampCurrentLocation(self.slave_id))
            self.ref_norm = self._norm_from_loc(loc)
            self.target_norm = self.ref_norm
            self.last_sent_norm = self.ref_norm
            self.active = False
            print(f"[gripper] reset_env open -> loc={loc}")
        except Exception as e:
            print(f"[gripper] reset open error: {e}")

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--web-port", type=int, default=5000)
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=5566)
    p.add_argument("--cert-file", default="certs/cert.pem", help="TLS certificate file path")
    p.add_argument("--key-file", default="certs/key.pem", help="TLS private key file path")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    root = os.path.join(os.path.dirname(__file__), "phone_teleop_web")
    script_dir = os.path.dirname(__file__)
    cert_file = args.cert_file if os.path.isabs(args.cert_file) else os.path.join(script_dir, args.cert_file)
    key_file = args.key_file if os.path.isabs(args.key_file) else os.path.join(script_dir, args.key_file)
    BridgeServer(
        root,
        args.web_port,
        args.udp_host,
        args.udp_port,
        cert_file,
        key_file,
    ).run()
