#!/usr/bin/env python3
"""Jodell EPG夹爪测试脚本（默认: /dev/ttyUSB0, id=9）。"""

from __future__ import annotations

import argparse
import importlib.util
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_ID = 9
DEFAULT_BAUD = 115200


def _check_runtime_deps() -> None:
    missing = []
    for module_name in ("serial", "modbus_tk"):
        if importlib.util.find_spec(module_name) is None:
            missing.append(module_name)

    if missing:
        raise ModuleNotFoundError(
            "缺少运行依赖: "
            + ", ".join(missing)
            + "\n请先安装：\n"
            + "  python3 -m pip install pyserial modbus-tk\n"
            + "若你之前安装过错误包 `serial`，请先卸载：\n"
            + "  python3 -m pip uninstall -y serial"
        )


def _import_epg_control():
    """优先使用已安装包；失败时从同目录whl加载。"""
    _check_runtime_deps()

    first_import_err: Exception | None = None
    try:
        from jodellSdk.jodellSdkDemo import EpgClawControl  # type: ignore

        return EpgClawControl
    except Exception as exc:
        first_import_err = exc

    wheel_path = Path(__file__).resolve().parent / "JodellTool-0.1.5-py3-none-any.whl"
    if wheel_path.exists():
        sys.path.insert(0, str(wheel_path))
        try:
            from jodellSdk.jodellSdkDemo import EpgClawControl  # type: ignore
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                f"已找到whl但导入失败: {exc}\n"
                "请安装依赖：python3 -m pip install pyserial modbus-tk"
            ) from exc
        except Exception as exc:
            raise RuntimeError(f"从whl导入 jodell SDK 失败: {exc}") from exc

        return EpgClawControl

    if first_import_err is not None:
        raise ModuleNotFoundError(
            "导入 jodell SDK 失败，且未找到本地whl："
            f"{first_import_err}\n"
            "请安装SDK：python3 -m pip install src/jodell/python/JodellTool-0.1.5-py3-none-any.whl"
        ) from first_import_err

    raise ModuleNotFoundError(
        "未找到jodell SDK。请先安装whl，或确保脚本目录下有 JodellTool-0.1.5-py3-none-any.whl"
    )


def ok(ret: Any) -> bool:
    return ret == 1


def show(ret: Any) -> str:
    return str(ret)


def read_and_print_basic(claw: Any, slave_id: int) -> None:
    print(f"software={show(claw.readSoftwareVersion(slave_id))}")
    print(f"temperature={show(claw.getDeviceCurrentTemperature(slave_id))}")
    print(f"voltage={show(claw.getDeviceCurrentVoltage(slave_id))}")
    print(f"clamp_state={show(claw.getClampCurrentState(slave_id))}")
    print(f"clamp_pos={show(claw.getClampCurrentLocation(slave_id))}")
    print(f"clamp_speed={show(claw.getClampCurrentSpeed(slave_id))}")
    print(f"clamp_torque={show(claw.getClampCurrentTorque(slave_id))}")


def run_keyboard_control(claw: Any, slave_id: int, torque: int, wait_s: float) -> None:
    max_speed = 255
    print("\n[交互控制]")
    print("按 o: 打开到最大 | 按 c: 关闭 | 按 q: 退出（无需回车）")

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            cmd = sys.stdin.read(1).lower()
            if cmd == "q":
                print("\n退出控制")
                break
            if cmd == "o":
                print("\n执行: 打开到最大")
                ret = claw.runWithParam(slave_id, 0, max_speed, torque)
                if not ok(ret):
                    print(f"打开失败: {ret}")
                    continue
                time.sleep(wait_s)
                print(f"clamp_state={show(claw.getClampCurrentState(slave_id))}")
                print(f"clamp_pos={show(claw.getClampCurrentLocation(slave_id))}")
                continue
            if cmd == "c":
                print("\n执行: 关闭")
                ret = claw.runWithParam(slave_id, 255, max_speed, torque)
                if not ok(ret):
                    print(f"关闭失败: {ret}")
                    continue
                time.sleep(wait_s)
                print(f"clamp_state={show(claw.getClampCurrentState(slave_id))}")
                print(f"clamp_pos={show(claw.getClampCurrentLocation(slave_id))}")
                continue
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def main() -> int:
    parser = argparse.ArgumentParser(description="Jodell EPG夹爪测试")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口，默认 {DEFAULT_PORT}")
    parser.add_argument("--id", type=int, default=DEFAULT_ID, help=f"从站ID，默认 {DEFAULT_ID}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"波特率，默认 {DEFAULT_BAUD}")
    parser.add_argument("--torque", type=int, default=80, help="动作力矩(0-255)")
    parser.add_argument("--wait", type=float, default=1.5, help="每次动作等待秒数")
    parser.add_argument("--no-motion", action="store_true", help="仅连通与状态读取，不执行开合")
    args = parser.parse_args()

    EpgClawControl = _import_epg_control()
    claw = EpgClawControl()

    print(f"连接串口: port={args.port}, baud={args.baud}, id={args.id}")
    ret = claw.serialOperation(args.port, args.baud, True)
    if not ok(ret):
        print(f"串口连接失败: {ret}")
        return 1

    try:
        ret = claw.enableClamp(args.id, True)
        if not ok(ret):
            raise RuntimeError(f"使能失败: {ret}")
        time.sleep(0.3)

        print("\n[基础信息]")
        read_and_print_basic(claw, args.id)

        if not args.no_motion:
            run_keyboard_control(claw, args.id, args.torque, args.wait)

        print("\n测试完成")
        return 0
    except Exception as exc:
        print(f"测试失败: {exc}")
        return 2
    finally:
        close_ret = claw.serialOperation(args.port, args.baud, False)
        if not ok(close_ret):
            print(f"断开串口异常: {close_ret}")


if __name__ == "__main__":
    raise SystemExit(main())
