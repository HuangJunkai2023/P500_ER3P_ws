#!/usr/bin/env python3
"""采集 xCore 机械臂末端 xyzrpy + RealSense 相机数据，并保存为 LeRobot 数据集格式。

用法示例：
python src/data_collect/collect_xcore_realsense_lerobot.py \
  --robot-ip 192.168.110.129 \
  --root ./datasets/xcore_demo \
  --repo-id local/xcore_er3p_demo \
  --episodes 5 \
  --episode-seconds 20
"""

from __future__ import annotations

import argparse
import os
import platform
import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np


def add_xcore_sdk_path(workspace_root: Path) -> None:
    """将 xCoreSDK_python 的路径加入 sys.path。"""
    sdk_root = workspace_root / "src" / "xcoresdk_python-v0.6.0"
    release_dir = sdk_root / "Release"
    linux_dir = release_dir / "linux"
    windows_dir = release_dir / "windows"

    for p in [sdk_root, release_dir, linux_dir, windows_dir]:
        p_str = str(p)
        if p_str not in sys.path:
            sys.path.append(p_str)


def load_xcore_sdk(workspace_root: Path):
    add_xcore_sdk_path(workspace_root)

    if platform.system() == "Windows":
        from Release.windows import xCoreSDK_python  # type: ignore
    elif platform.system() == "Linux":
        from Release.linux import xCoreSDK_python  # type: ignore
    else:  # pragma: no cover
        raise RuntimeError(f"不支持的系统: {platform.system()}")

    return xCoreSDK_python


class XCoreRobotReader:
    def __init__(self, xcore_sdk, robot_ip: str, robot_model: str = "er3pro"):
        self.xcore_sdk = xcore_sdk
        self.robot_model = robot_model.lower()
        self.ec = {}
        self.robot = self._create_robot(xcore_sdk, robot_ip, self.robot_model)
        self._connect_robot(robot_ip)

    def _create_robot(self, xcore_sdk, robot_ip: str, robot_model: str):
        model = robot_model.lower()
        if model in ("er3pro", "er7pro", "xmateerpro"):
            return xcore_sdk.xMateErProRobot()
        if model in ("xmate", "xmaterobot"):
            return xcore_sdk.xMateRobot(robot_ip)
        if model in ("cr5", "xmatecr5"):
            return xcore_sdk.xMateCr5Robot()
        raise ValueError(f"不支持的 robot_model: {robot_model}")

    def _connect_robot(self, robot_ip: str) -> None:
        if self.robot_model in ("er3pro", "er7pro", "xmateerpro", "cr5", "xmatecr5"):
            self.robot.connectToRobot(robot_ip)
            return
        if self.robot_model in ("xmate", "xmaterobot"):
            self.robot.connectToRobot(self.ec)
            return
        raise ValueError(f"不支持的 robot_model: {self.robot_model}")

    def get_xyzrpy(self) -> np.ndarray:
        cart_posture = self.robot.cartPosture(self.xcore_sdk.CoordinateType.endInRef, self.ec)
        xyz = np.asarray(cart_posture.trans, dtype=np.float32)
        rpy = np.asarray(cart_posture.rpy, dtype=np.float32)
        return np.concatenate([xyz, rpy], axis=0)

    def close(self) -> None:
        try:
            self.robot.disconnectFromRobot(self.ec)
        except Exception:
            pass


class RealSenseReader:
    def __init__(self, width: int, height: int, fps: int, warmup_frames: int = 30):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:  # pragma: no cover
            raise ImportError("未安装 pyrealsense2，请先安装 librealsense 与 pyrealsense2") from exc

        self.rs = rs
        self.pipeline = self.rs.pipeline()
        self.config = self.rs.config()
        self.config.enable_stream(self.rs.stream.color, width, height, self.rs.format.rgb8, fps)
        self.profile = self.pipeline.start(self.config)
        self.width = width
        self.height = height
        self.fps = fps

        for _ in range(max(0, warmup_frames)):
            self.pipeline.wait_for_frames()

    def get_rgb(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("未获取到 RealSense 彩色帧")
        color = np.asanyarray(color_frame.get_data())
        if color.shape != (self.height, self.width, 3):
            raise RuntimeError(f"相机帧尺寸异常: {color.shape}, 期望 {(self.height, self.width, 3)}")
        return color.astype(np.uint8)

    def close(self) -> None:
        self.pipeline.stop()


def build_features(height: int, width: int) -> dict:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["x", "y", "z", "roll", "pitch", "yaw"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["dx", "dy", "dz", "droll", "dpitch", "dyaw"],
        },
        "observation.images.realsense": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
    }


def record_one_episode(
    dataset: LeRobotDataset,
    robot_reader: XCoreRobotReader,
    camera_reader: RealSenseReader,
    fps: int,
    seconds: float,
    task: str,
) -> Tuple[int, float]:
    period = 1.0 / fps
    steps = int(seconds * fps)

    prev_pose = None
    start = time.time()

    for step in range(steps):
        t0 = time.time()

        pose = robot_reader.get_xyzrpy()
        rgb = camera_reader.get_rgb()

        if prev_pose is None:
            action = np.zeros(6, dtype=np.float32)
        else:
            action = (pose - prev_pose).astype(np.float32)

        frame = {
            "observation.state": pose.astype(np.float32),
            "action": action,
            "observation.images.realsense": rgb,
            "task": task,
        }
        dataset.add_frame(frame)
        prev_pose = pose

        elapsed = time.time() - t0
        sleep_t = period - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    dataset.save_episode()
    return steps, time.time() - start


def parse_args() -> argparse.Namespace:
    default_robot_ip = os.environ.get("XCORE_ROBOT_IP", "192.168.0.160")
    parser = argparse.ArgumentParser(description="采集 xCore + RealSense 到 LeRobot 数据集")
    parser.add_argument(
        "--robot-ip",
        default=default_robot_ip,
        help=f"xCore 机械臂控制器 IP（默认: {default_robot_ip}，可用环境变量 XCORE_ROBOT_IP 覆盖）",
    )
    parser.add_argument(
        "--robot-model",
        default="er3pro",
        choices=["er3pro", "er7pro", "xmate", "cr5"],
        help="机器人机型：ER3Pro/ER7Pro 推荐 er3pro 或 er7pro",
    )
    parser.add_argument("--workspace-root", default=str(Path(__file__).resolve().parents[2]), help="工作区根目录")
    parser.add_argument("--root", required=True, help="LeRobot 数据集根目录（本地路径）")
    parser.add_argument("--repo-id", default="local/xcore_er3p_dataset", help="LeRobot repo_id（本地可用 local/xxx）")
    parser.add_argument("--robot-type", default="xcore_er3p", help="写入到 LeRobot metadata 的 robot_type")

    parser.add_argument("--fps", type=int, default=30, help="采集频率")
    parser.add_argument("--episodes", type=int, default=1, help="采集 episode 数量")
    parser.add_argument("--episode-seconds", type=float, default=20.0, help="每个 episode 时长（秒）")
    parser.add_argument("--task", default="xcore_teleop", help="LeRobot 任务文本")
    parser.add_argument("--prompt", default=None, help="任务提示词（优先于 --task，用于写入 LeRobot task）")

    parser.add_argument("--rs-width", type=int, default=640, help="RealSense 图像宽")
    parser.add_argument("--rs-height", type=int, default=480, help="RealSense 图像高")
    parser.add_argument("--rs-fps", type=int, default=30, help="RealSense 采集帧率")
    parser.add_argument("--warmup-frames", type=int, default=30, help="相机预热帧数")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    workspace_root = Path(args.workspace_root).resolve()
    output_root = Path(args.root).resolve()

    if output_root.exists():
        raise FileExistsError(f"输出目录已存在，请使用新目录: {output_root}")

    xcore_sdk = load_xcore_sdk(workspace_root)
    robot_reader = XCoreRobotReader(xcore_sdk, args.robot_ip, args.robot_model)

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:  # pragma: no cover
        robot_reader.close()
        raise ImportError("未安装 lerobot，请先 `pip install lerobot`") from exc

    camera_reader = RealSenseReader(
        width=args.rs_width,
        height=args.rs_height,
        fps=args.rs_fps,
        warmup_frames=args.warmup_frames,
    )

    features = build_features(args.rs_height, args.rs_width)
    task_text = args.prompt if args.prompt is not None and args.prompt.strip() else args.task

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=output_root,
        robot_type=args.robot_type,
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=2,
        batch_encoding_size=1,
    )

    try:
        print(f"开始采集: episodes={args.episodes}, seconds={args.episode_seconds}, fps={args.fps}")
        for ep in range(args.episodes):
            steps, cost = record_one_episode(
                dataset=dataset,
                robot_reader=robot_reader,
                camera_reader=camera_reader,
                fps=args.fps,
                seconds=args.episode_seconds,
                task=task_text,
            )
            print(f"Episode {ep} 完成: {steps} 帧, 用时 {cost:.2f}s")
    finally:
        try:
            dataset.finalize()
        finally:
            robot_reader.close()
            camera_reader.close()

    print(f"采集完成，LeRobot 数据集已保存到: {output_root}")


if __name__ == "__main__":
    main()
