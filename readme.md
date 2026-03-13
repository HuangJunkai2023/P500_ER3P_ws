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

### 1. 训练前确认

当前工作区已经有：

- 数据目录：`src/tidybot2/data/demos`
- 训练仓库：`src/diffusion_policy`
- Conda 环境：`robodiff`

另外，`src/diffusion_policy` 需要已经包含 `tidybot2` 的 patch 改动，即：

- `diffusion_policy/config/task/square_image_abs.yaml` 使用 `base_image`、`wrist_image`、`base_pose`、`arm_pos`、`arm_quat`、`gripper_pos`
- 数据路径格式为 `data/${task.name}.hdf5`

本工作区当前实际使用的任务名是 `demos`，对应配置文件：

```yaml
# src/diffusion_policy/diffusion_policy/config/task/square_image_abs.yaml
name: demos
```

### 2. 将采集数据转换为 HDF5

这一步实际是在 `robodiff` 环境中执行的，因为转换脚本依赖 `cv2`。

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws
source /home/huaxi/miniconda3/etc/profile.d/conda.sh
conda activate robodiff
python src/tidybot2/convert_to_robomimic_hdf5.py \
  --input-dir src/tidybot2/data/demos \
  --output-path src/diffusion_policy/data/demos.hdf5
```

生成结果：

- HDF5 文件：`src/diffusion_policy/data/demos.hdf5`

### 3. 启动训练

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/diffusion_policy
source /home/huaxi/miniconda3/etc/profile.d/conda.sh
conda activate robodiff
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python train.py --config-name=train_diffusion_unet_real_hybrid_workspace
```

说明：

- `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` 是当前环境里必须加的兼容项。
- 原因是升级 `wandb` 后，`tensorboard/protobuf` 组合会触发 `Descriptors cannot be created directly` 错误。
- 如果后续把环境重新整理到兼容版本，这个环境变量可以再评估是否移除。

### 4. wandb 登录

训练前需要先在 `robodiff` 环境里完成一次登录：

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/diffusion_policy
source /home/huaxi/miniconda3/etc/profile.d/conda.sh
conda activate robodiff
wandb login
```


### 5. 训练输出位置

- 日志与模型输出目录：
  `src/diffusion_policy/data/outputs/<日期>/<时间>_train_diffusion_unet_hybrid_demos/`
- 可用于推理的模型示例：
  `src/diffusion_policy/data/outputs/.../checkpoints/epoch=0500-train_loss=xxx.ckpt`

本次实际启动的训练目录示例：

- `src/diffusion_policy/data/outputs/2026.03.13/16.56.14_train_diffusion_unet_hybrid_demos/`

### 6. （可选）启动策略推理服务

```bash
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src/diffusion_policy
source /home/huaxi/miniconda3/etc/profile.d/conda.sh
conda activate robodiff
python /home/huaxi/scu_robotics/P500_ER3P_ws/src/tidybot2/policy_server.py \
  --ckpt-path data/outputs/<日期>/<时间>_train_diffusion_unet_hybrid_demos/checkpoints/<你的ckpt>.ckpt
```
