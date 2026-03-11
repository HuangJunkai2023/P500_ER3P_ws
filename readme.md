# P500 + ER3Pro 使用说明

## 快速开始（先做这一步）

使用 ROS1 Docker（推荐这一种方式）：

```bash
cd /home/hx/P500_ER3P_ws && docker compose run --rm ros1
```

进入容器后运行：

```bash
python3 /workspace/src/p500/docker_cmd_vel_server.py --udp-port 15000
```

退出容器：`Ctrl + D`

## 机械臂 SDK

- 下载地址：http://sw.rokae.com:8800/?dir=xcore/SDK/v3.1

## 手机示教（Jetson）

### 1. 编译 arm bridge

```bash
cd xCoreSDK_cpp-v0.7.1
cmake -S . -B build -DXCORE_USE_XMATE_MODEL=OFF
cmake --build build --target arm_bridge -j4
```

### 2. 启动示教

```bash
conda activate tidybot2
```

先确认 `src/tidybot2/constants.py` 中底盘后端配置为：

```python
ENABLE_BASE = True
BASE_BACKEND = 'ros1_udp'
BASE_CMD_UDP_HOST = '127.0.0.1'
BASE_CMD_UDP_PORT = 15000

# 夹爪：通过机械臂末端 RS485 控制 Jodell EPG
ER3PRO_ENABLE_GRIPPER = True
ER3PRO_GRIPPER_BACKEND = 'rs485_epg'
ER3PRO_GRIPPER_RS485_SLAVE_ID = 9
```

终端 1（ROS1 Docker 内）：

```bash
python /workspace/src/p500/docker_cmd_vel_server.py --udp-port 15000
```

终端 2（工作区 `src/tidybot2`）：

```bash
python base_server.py
```

终端 3（工作区 `src/tidybot2`）：

```bash
python arm_server.py
```

终端 4（工作区 `src/tidybot2`）：

```bash
python main.py --teleop --ssl --save
```

说明：
- `base_server.py` 接收 `tidybot` 的 `base_pose` 目标并转换为速度命令。
- `docker_cmd_vel_server.py` 在 ROS1 中发布到底盘 `/cmd_vel`。
- 如果 ROS1 Docker 不在本机，修改 `BASE_CMD_UDP_HOST` 为 ROS1 侧可达 IP。

## 模型训练

### 1. 将采集数据转换为 HDF5

假设数据目录为 `src/tidybot2/data/demos`：

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/tidybot2
python convert_to_robomimic_hdf5.py \
	--input-dir data/demos \
	--output-path ../diffusion_policy/data/p500_er3p_v1.hdf5
```

### 2. 在 diffusion_policy 中启动训练

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/diffusion_policy
conda activate robodiff
python train.py \
	--config-name=train_diffusion_unet_real_hybrid_workspace \
	task=square_image_abs \
	task.name=p500_er3p_v1
```

### 3. 训练输出位置

- 日志与模型输出目录：
	`src/diffusion_policy/data/outputs/<日期>/<时间>_train_diffusion_unet_hybrid_p500_er3p_v1/`
- 可用于推理的模型示例：
	`src/diffusion_policy/data/outputs/.../checkpoints/epoch=0500-train_loss=xxx.ckpt`

### 4. （可选）启动策略推理服务

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/diffusion_policy
conda activate robodiff
python /home/huaxi/scu_robotics/P500_ER3P_ws/src/tidybot2/policy_server.py \
	--ckpt-path data/outputs/<日期>/<时间>_train_diffusion_unet_hybrid_p500_er3p_v1/checkpoints/<你的ckpt>.ckpt
```
