"""ER3Pro 末端工具接口 IO 输出控制夹爪。

控制映射（来自《8.1 DI 输入控制》）：
- DI1=0, DI2=0/1: 清除使能，恢复空闲状态
- DI1=1, DI2=0: 使能 / 执行预设参数1
- DI1=1, DI2=1: 执行预设参数2

说明：
- 本脚本通过机械臂 DO 输出去驱动夹爪 DI1/DI2。
- 默认将 DO0 -> DI1，DO1 -> DI2，可通过命令行改端口。
"""

from __future__ import annotations

import argparse
import importlib
import platform
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Tuple


def _import_xcore_sdk():
	"""动态导入 xCoreSDK_python，兼容当前工程目录结构。"""
	system_name = platform.system()
	if system_name == "Windows":
		module_name = "Release.windows.xCoreSDK_python"
	elif system_name == "Linux":
		module_name = "Release.linux.xCoreSDK_python"
	else:
		raise RuntimeError(f"不支持的系统: {system_name}")

	try:
		return importlib.import_module(module_name)
	except ModuleNotFoundError:
		current_file = Path(__file__).resolve()
		candidate_roots = [
			current_file.parents[1] / "xcoresdk_python-v0.6.0",
			current_file.parents[2] / "xcoresdk_python-v0.6.0",
		]
		for candidate in candidate_roots:
			if candidate.exists():
				candidate_str = str(candidate)
				if candidate_str not in sys.path:
					sys.path.insert(0, candidate_str)

		return importlib.import_module(module_name)


xCoreSDK_python = _import_xcore_sdk()


def create_and_connect_robot(ip: str):
	"""按 ER3Pro 机型创建并连接机器人实例。"""
	# ER3 Pro / ER7 Pro 机型：xMateErProRobot + connectToRobot(ip)
	if hasattr(xCoreSDK_python, "xMateErProRobot"):
		robot = xCoreSDK_python.xMateErProRobot()
		robot.connectToRobot(ip)
		return robot

	# 兼容旧SDK：回退到 xMateRobot(ip)
	if hasattr(xCoreSDK_python, "xMateRobot"):
		return xCoreSDK_python.xMateRobot(ip)

	raise RuntimeError("当前 xCoreSDK_python 未找到可用机器人类型（xMateErProRobot/xMateRobot）")


class ER3ProGripperIOController:
	def __init__(
		self,
		robot,
		board: int = 2,
		di1_do_port: int = 0,
		di2_do_port: int = 1,
		settle_time: float = 0.05,
	) -> None:
		self.robot = robot
		self.board = board
		self.di1_do_port = di1_do_port
		self.di2_do_port = di2_do_port
		self.settle_time = settle_time

	@staticmethod
	def _check_ec(operation: str, ec: dict) -> None:
		code = ec.get("code")
		if code is not None and code != 0:
			raise RuntimeError(f"{operation} 失败: code={code}, ec={ec}")

	def _set_do(self, port: int, state: bool) -> None:
		ec = {}
		self.robot.setDO(self.board, port, bool(state), ec)
		self._check_ec(f"setDO(board={self.board}, port={port}, state={state})", ec)

	def _get_do(self, port: int) -> bool:
		ec = {}
		value = self.robot.getDO(self.board, port, ec)
		self._check_ec(f"getDO(board={self.board}, port={port})", ec)
		return bool(value)

	def set_gripper_di(self, di1: int, di2: int) -> None:
		self._set_do(self.di1_do_port, bool(di1))
		self._set_do(self.di2_do_port, bool(di2))
		if self.settle_time > 0:
			time.sleep(self.settle_time)

	def clear_enable(self) -> None:
		self.set_gripper_di(0, 0)

	def enable_or_preset1(self) -> None:
		self.set_gripper_di(1, 0)

	def preset2(self) -> None:
		self.set_gripper_di(1, 1)

	def read_output_state(self) -> Tuple[bool, bool]:
		di1_state = self._get_do(self.di1_do_port)
		di2_state = self._get_do(self.di2_do_port)
		return di1_state, di2_state


def execute_action(controller: ER3ProGripperIOController, action: str) -> None:
	if action == "clear":
		controller.clear_enable()
	elif action == "enable":
		controller.enable_or_preset1()
	elif action in ("preset1", "open"):
		controller.enable_or_preset1()
	elif action in ("preset2", "close"):
		controller.preset2()
	else:
		raise ValueError(f"不支持的 action: {action}")


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="ER3Pro 末端IO控制夹爪")
	parser.add_argument("--ip", default="192.168.0.160", help="机械臂控制器IP，默认 192.168.0.160")
	parser.add_argument(
		"--action",
		default=None,
		choices=["clear", "enable", "preset1", "preset2", "open", "close"],
		help="执行单次动作；不传时进入键盘交互模式",
	)
	parser.add_argument("--board", type=int, default=2, help="IO板编号，默认 2")
	parser.add_argument("--di1-port", type=int, default=0, help="对应夹爪 DI1 的 DO 端口")
	parser.add_argument("--di2-port", type=int, default=1, help="对应夹爪 DI2 的 DO 端口")
	parser.add_argument("--settle-time", type=float, default=0.05, help="每次输出后等待秒数")
	parser.add_argument("--repeat", type=int, default=1, help="重复发送次数")
	parser.add_argument("--interval", type=float, default=0.2, help="重复发送间隔秒数")
	parser.add_argument(
		"--clear-before",
		action="store_true",
		help="在 preset1/preset2 前先发一次 clear，适合上电后初始化",
	)
	parser.add_argument("--no-interactive", action="store_true", help="关闭交互模式；仅执行 --action")
	parser.add_argument("--print-state", action="store_true", help="执行后打印 DO 状态")
	return parser.parse_args()


def _read_key_non_blocking(timeout: float = 0.1) -> str:
	fd = sys.stdin.fileno()
	old_attrs = termios.tcgetattr(fd)
	try:
		tty.setcbreak(fd)
		ready, _, _ = select.select([sys.stdin], [], [], timeout)
		if ready:
			return sys.stdin.read(1).lower()
		return ""
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def interactive_loop(controller: ER3ProGripperIOController, print_state: bool = False) -> None:
	print("已进入键盘控制: [1/o]=打开(预设1), [2/c]=关闭(预设2), [e]=使能, [x]=清除使能, [q]=退出")
	while True:
		key = _read_key_non_blocking(0.2)
		if not key:
			continue

		if key in ("1", "o"):
			execute_action(controller, "preset1")
			print("执行: 预设1(打开)")
		elif key in ("2", "c"):
			execute_action(controller, "preset2")
			print("执行: 预设2(关闭)")
		elif key == "e":
			execute_action(controller, "enable")
			print("执行: 使能")
		elif key == "x":
			execute_action(controller, "clear")
			print("执行: 清除使能")
		elif key == "q":
			print("退出键盘控制")
			break
		else:
			print(f"忽略按键: {key}")

		if print_state:
			di1_state, di2_state = controller.read_output_state()
			print(f"DO状态: DI1(port={controller.di1_do_port})={int(di1_state)}, DI2(port={controller.di2_do_port})={int(di2_state)}")


def main() -> None:
	args = parse_args()

	robot = create_and_connect_robot(args.ip)
	controller = ER3ProGripperIOController(
		robot=robot,
		board=args.board,
		di1_do_port=args.di1_port,
		di2_do_port=args.di2_port,
		settle_time=args.settle_time,
	)

	# 按需求：程序启动后先执行使能
	controller.enable_or_preset1()
	print("启动已执行使能: DI1=1, DI2=0")

	if args.clear_before and args.action in {"preset1", "preset2", "open", "close"}:
		controller.clear_enable()
		time.sleep(max(args.interval, 0.05))

	if args.action is not None:
		repeat_times = max(1, args.repeat)
		for index in range(repeat_times):
			execute_action(controller, args.action)
			if index != repeat_times - 1 and args.interval > 0:
				time.sleep(args.interval)

		if args.print_state:
			di1_state, di2_state = controller.read_output_state()
			print(f"DO状态: DI1(port={args.di1_port})={int(di1_state)}, DI2(port={args.di2_port})={int(di2_state)}")

		print(f"动作执行完成: action={args.action}, board={args.board}, ip={args.ip}")
		if args.no_interactive:
			return

	if args.no_interactive and args.action is None:
		print("未提供 --action 且已设置 --no-interactive，程序结束。")
		return

	interactive_loop(controller, print_state=args.print_state)


if __name__ == "__main__":
	main()
