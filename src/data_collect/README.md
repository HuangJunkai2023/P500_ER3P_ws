# xCore + RealSense 采集到 LeRobot

本目录提供一个最小可运行采集脚本：
- 机械臂：读取 xCore 末端 `xyzrpy`
- 相机：读取 RealSense 彩色图
- 存储：按 LeRobot 官方数据格式（`meta/` + `data/` + `videos/`）落盘

## 1. 安装依赖

```bash
pip install -r src/data_collect/requirements.txt
```

> 需要先正确安装并可用 `pyrealsense2`（通常依赖 librealsense）。

## 2. 运行采集

```bash
python src/data_collect/collect_xcore_realsense_lerobot.py \
  --robot-model er3pro \
  --root ./datasets/xcore_er3p_demo \
  --repo-id local/xcore_er3p_demo \
  --prompt "抓取并放置目标到托盘" \
  --episodes 5 \
  --episode-seconds 20 \
  --fps 30

其中 `--robot-ip` 可省略，默认使用 `192.168.0.160`。
也可通过环境变量覆盖：

```bash
export XCORE_ROBOT_IP=192.168.0.160
```
```

## 3. 当前写入的 LeRobot 特征

- 机型参数：`--robot-model`，默认 `er3pro`（适配 ER3 Pro / ER7 Pro）
- 任务文本：`--prompt` 优先于 `--task`，最终写入 LeRobot 的 `task`

- `observation.state`: 6 维，`[x, y, z, roll, pitch, yaw]`
- `action`: 6 维，当前帧与上一帧的位姿差分 `dxyzrpy`
- `observation.images.realsense`: RealSense RGB 视频帧

## 4. 输出结果

输出目录示例：

```text
<root>/
  meta/
    info.json
    episodes/chunk-000/file-000.parquet
    stats.json
    tasks.jsonl
  data/chunk-000/file-000.parquet
  videos/chunk-000/observation.images.realsense/episode_000000.mp4
```

## 5. 注意事项

- 采集脚本要求输出目录"不存在"，避免覆盖旧数据。
- 若需要保留真实控制动作，可把 `action` 替换成你自己的控制命令。
- 默认仅写入 RealSense 彩色帧；如果你要保存深度图，可在此脚本上扩展一个额外特征。
