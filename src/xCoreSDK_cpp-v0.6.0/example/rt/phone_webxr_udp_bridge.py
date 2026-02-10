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

        tpl = os.path.join(web_root, "templates")
        sta = os.path.join(web_root, "static")
        self.app = Flask(__name__, template_folder=tpl, static_folder=sta)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
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

            # enabled = 1 only while touch teleop is active
            mode = 1 if data.get("teleop_mode") == "arm" else 0
            enabled = 1 if "teleop_mode" in data and mode == 1 else 0
            if enabled == 1:
                self.last_mode = "arm(enabled)"
            elif data.get("teleop_mode") == "base":
                self.last_mode = "base"
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

            pkt = f"{px},{py},{pz},{qx},{qy},{qz},{qw},{enabled},{mode}\n".encode("utf-8")
            self.udp_sock.sendto(pkt, self.udp_addr)
            self.tx_count += 1

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
