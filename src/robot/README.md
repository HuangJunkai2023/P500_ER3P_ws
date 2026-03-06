# P500 + ER3P Robot Interface for VLA Development

> **目标读者**：在 RTX 2080 Ti 22G 主机上开发 VLA 模型的研究人员  
> 本文档只关注主机侧接口 `host_vla_client.py`。  
> 底盘、机械臂、夹爪、相机均由 Jetson 统一管理，**开发者无需了解 Jetson 内部细节**。

---

## 目录

1. [系统概览](#系统概览)
2. [安装依赖](#安装依赖)
3. [五分钟快速上手](#五分钟快速上手)
4. [RobotEnv API](#robotenv-api)
5. [Observation 说明](#observation-说明)
6. [Action 说明](#action-说明)
7. [与训练框架集成](#与训练框架集成)
8. [注意事项与安全限制](#注意事项与安全限制)

---

## 系统概览

```
┌──────────────────────────────────────────────┐
│          VLA 主机（RTX 2080 Ti 22G）           │
│                                              │
│   obs = env.reset()                         │
│   action = vla_policy(obs)                  │
│   obs, info = env.step(action)              │
│                                              │
│   from host_vla_client import RobotEnv      │
└─────────────────┬────────────────────────────┘
                  │  ZMQ REQ/REP  TCP 7788
                  │  msgpack · JPEG
                  ▼
┌──────────────────────────────────────────────┐
│               Jetson（机器人主控）              │
│  ├─ P500 底盘   [vx, vy, wz]  m/s · rad/s  │
│  ├─ ER3P 机械臂 [dj0..dj5]   rad 增量       │
│  ├─ 夹爪        0.0 ~ 1.0                   │
│  └─ 双路 RGB 相机 → JPEG                    │
└──────────────────────────────────────────────┘
```

主机通过 **ZMQ REQ/REP** 与 Jetson 交互，每个 `step()` 调用：
- **发送**：底盘速度 + 机械臂关节增量 + 夹爪值
- **接收**：当前关节角 + 底盘速度 + 双路 RGB 图像 + 时间戳

局域网典型往返延迟：**10 ~ 30 ms**（不含图像解码）。

---

## 安装依赖

```bash
pip install pyzmq msgpack msgpack-numpy numpy opencv-python
```

在使用前确认 Jetson 已启动服务端（联系硬件负责人）：

```
Jetson 服务地址: tcp://<jetson_ip>:7788
```

---

## 五分钟快速上手

```python
from host_vla_client import RobotEnv
import numpy as np

# 1. 连接机器人
env = RobotEnv(jetson_ip="192.168.1.200")

# 2. 获取初始观测
obs = env.reset()
print(obs)
# Observation(
#   joint_pos   = [0.0, 0.52, 1.05, 0.0, 1.57, 0.0],   rad
#   chassis_vel = [0.0, 0.0, 0.0],                       m/s · rad/s
#   cam0.shape  = (480, 640, 3),
#   cam1.shape  = (480, 640, 3),
#   timestamp   = 1741234567.123
# )

# 3. 推理并执行动作
for step in range(100):
    action = {
        "chassis"   : [0.1, 0.0, 0.0],           # 以 0.1 m/s 前进
        "arm_delta" : np.zeros(6, dtype=float),   # 机械臂保持不动
        "gripper"   : 1.0,                         # 夹爪打开
    }
    obs, info = env.step(action)
    print(f"step {step} | latency {info['latency_ms']:.1f} ms | "
          f"joints {np.round(obs.joint_pos, 3)}")

# 4. 关闭（自动发送停止指令）
env.close()
```

---

## RobotEnv API

### 初始化

```python
from host_vla_client import RobotEnv

env = RobotEnv(
    jetson_ip  = "192.168.1.200",  # Jetson IP，联系硬件负责人确认
    port       = 7788,              # ZMQ 端口（默认不需要改）
    timeout_ms = 5000,              # 单步通信超时，ms
)
```

---

### `env.reset() → Observation`

发送零动作，刷新并返回初始观测。**每个 episode 开始时必须调用**。

```python
obs = env.reset()
```

---

### `env.step(action) → (Observation, info)`

执行一步动作，同步返回新的观测。`info` 包含 `latency_ms`（主机与 Jetson 的往返延迟）。

```python
obs, info = env.step({
    "chassis"     : [0.1, 0.0, 0.0],         # [vx, vy, wz]
    "arm_delta"   : [0, 0, 0, 0.05, 0, 0],   # 各关节增量 rad
    "gripper"     : 0.0,                       # 0.0=关闭
    "arm_velocity": 300,                       # 可选，默认 300
})
print(info["latency_ms"])   # 往返延迟，ms
```

---

### 便捷方法

只需控制单一子系统时使用，其余部分自动归零/保持静止：

```python
# 仅移动底盘
env.send_chassis(vx=0.15, vy=0.0, wz=0.3)

# 仅移动机械臂（增量，rad）
delta = np.zeros(6)
delta[3] = 0.05
obs = env.send_arm_delta(delta, velocity=300)

# 仅控制夹爪
obs = env.set_gripper(0.0)   # 0.0=关闭, 1.0=打开

# 急停（所有执行器立即归零速）
env.stop()
```

---

### 上下文管理器（推荐写法）

```python
with RobotEnv(jetson_ip="192.168.1.200") as env:
    obs = env.reset()
    for action in vla_policy.rollout(obs):
        obs, info = env.step(action)
# 退出 with 块后自动 close()，底盘和机械臂安全停止
```

---

### 连通性测试

```bash
# 在主机终端运行，验证与 Jetson 的通信是否正常
python3 host_vla_client.py --jetson-ip 192.168.1.200 --steps 10
```

---

## Observation 说明

每次 `step()` / `reset()` 返回一个 `Observation` 对象：

| 属性 | numpy 类型 | 形状 | 单位 | 描述 |
|------|-----------|------|------|------|
| `joint_pos` | `float32` | `(6,)` | rad | 机械臂 6 轴当前关节角，顺序对应 ER3P 物理轴 1~6 |
| `chassis_vel` | `float32` | `(3,)` | m/s, m/s, rad/s | 底盘速度 `[vx, vy, wz]`（差速底盘 vy≈0） |
| `cam0` | `uint8` | `(480, 640, 3)` | RGB | 相机 0 图像，通道顺序为 RGB |
| `cam1` | `uint8` | `(480, 640, 3)` | RGB | 相机 1 图像，通道顺序为 RGB |
| `timestamp` | `float` | scalar | s | Jetson 侧 Unix 时间戳 |

```python
# 转为 dict（直接传给模型 forward）
obs_dict = obs.as_dict()

# 图像转 tensor 示例
import torch
img = torch.from_numpy(obs.cam0).permute(2, 0, 1).float() / 255.0  # (3, H, W)
```

---

## Action 说明

`env.step()` 接受一个 Python `dict`，`list`、`tuple`、`np.ndarray` 均可作为字段值：

```python
action = {
    # ── 底盘 ──────────────────────────────────────────────
    "chassis": [vx, vy, wz],
    #           ↑    ↑    ↑
    #          m/s  m/s  rad/s
    #  P500 为差速底盘，vy 填 0.0；wz 正值为左转

    # ── 机械臂 ────────────────────────────────────────────
    "arm_delta": [dj0, dj1, dj2, dj3, dj4, dj5],
    #  单位 rad，表示相对当前关节角的增量
    #  Jetson 侧自动叠加并 clip 至软限位，不会超限

    # ── 夹爪 ──────────────────────────────────────────────
    "gripper": float,    # 0.0 = 完全关闭 · 1.0 = 完全打开

    # ── 可选 ──────────────────────────────────────────────
    "arm_velocity": int, # 机械臂运动速度 0~1000（默认 300）
}
```

**推荐范围**

| 参数 | 建议值 | 硬件上限 |
|------|--------|----------|
| `vx` | ≤ 0.2 m/s | 0.5 m/s |
| `wz` | ≤ 0.4 rad/s | 1.0 rad/s |
| 单步 `arm_delta[i]` | ≤ 0.05 rad | 0.3 rad |
| `arm_velocity` | 200 ~ 400 | 800 |

---

## 与训练框架集成

### Diffusion Policy / ACT 推理

```python
from host_vla_client import RobotEnv
import numpy as np, torch

env = RobotEnv(jetson_ip="192.168.1.200")
obs = env.reset()

policy.eval()
with torch.no_grad():
    for _ in range(episode_length):
        model_input = {
            "joint_pos": torch.tensor(obs.joint_pos).unsqueeze(0),
            "image"    : torch.from_numpy(obs.cam0).permute(2, 0, 1).float() / 255.0,
        }
        pred = policy.predict(model_input)

        obs, info = env.step({
            "chassis"   : pred["chassis"].tolist(),
            "arm_delta" : pred["arm_delta"].tolist(),
            "gripper"   : float(pred["gripper"]),
        })

env.close()
```

### 数据录制

```python
import pickle, time

trajectory = []
with RobotEnv(jetson_ip="192.168.1.200") as env:
    obs = env.reset()
    for step in range(200):
        action = teleop_get_action()            # 遥控获取动作
        next_obs, info = env.step(action)
        trajectory.append({
            "obs"    : obs.as_dict(),
            "action" : action,
            "latency": info["latency_ms"],
        })
        obs = next_obs

with open(f"traj_{int(time.time())}.pkl", "wb") as f:
    pickle.dump(trajectory, f)
```

---

## 注意事项与安全限制

- **先 `reset()`，再 `step()`**：跳过 `reset()` 可能导致机械臂从未知位置开始运动。

- **超时处理**：`step()` 超过 `timeout_ms`（默认 5000 ms）会抛出 `TimeoutError`：
  ```python
  try:
      obs, _ = env.step(action)
  except TimeoutError:
      env.stop()
      break
  ```

- **推荐调用频率**：10 ~ 30 Hz。过高频率（> 50 Hz）受限于网络往返和机械臂控制器响应。

- **急停**：任何异常情况下调用 `env.stop()` 或退出 `with` 块，底盘和机械臂将安全停止。

- **相机分辨率**：默认 640 × 480 RGB JPEG，如需修改请联系硬件负责人调整 Jetson 启动参数。

- **单步增量上限**：模型输出的 `arm_delta[i]` 建议限幅至 ±0.05 rad，避免机械臂冲击。
