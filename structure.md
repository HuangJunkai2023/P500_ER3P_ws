# P500_ER3P_ws 模块关系图

基于本地 checkout 的工程地图。箭头表示调用、通信或数据流；各路径是否启用取决于配置与启动入口。

```text
P500_ER3P_ws
  |
  +-- 固定子模块 commit、Docker 配置、硬件 SDK、部署文档
  |
  +-- 在线运行：tidybot2
  |     |
  |     +-- 手机 / 手柄 / UArm
  |     |       |
  |     |       v
  |     |   policies.py：遥操作策略
  |     |       |
  |     +-------+
  |             v
  |          main.py
  |       组织观测、动作、记录
  |             |
  |             +-- RemotePolicy <--- ZMQ ---> 推理服务
  |             |                             |
  |             |                     diffusion_policy
  |             |                     模型 + checkpoint
  |             v
  |         real_env.py
  |             |
  |             +-- cameras.py <------------ 双相机
  |             |
  |             +-- RPC --> base_server.py
  |             |               |
  |             |               +-- UDP --> p500/docker_cmd_vel_server.py
  |             |                              |
  |             |                              +-- ROS1 /cmd_vel --> P500
  |             |
  |             +-- RPC --> arm_server.py
  |                             |
  |                             +-- 子进程 --> arm_bridge.cpp
  |                                              |
  |                                              +-- xCore C++ SDK --> ER3Pro
  |                                                                      |
  |                                                                      +-- 末端 RS485 --> Jodell
  |
  +-- 独立 UArm 采集路径
  |     |
  |     +-- tidybot2/record_lerobot_uarm_er3pro.py
  |             |
  |             +-- 启动/读取状态 --> uarm_er3pro_rt.cpp
  |             |                       ^
  |             |                       |
  |             |                  UArm 串口输入
  |             |
  |             |   uarm_er3pro_rt.cpp -- xCore C++ SDK --> ER3Pro / 夹爪
  |             |
  |             +-- 相机图像 + 状态/动作 --> LeRobot 数据集
  |
  +-- 离线数据与训练
  |     |
  |     +-- Episode 数据 --> HDF5 转换 -----------+
  |     |                                        |
  |     +-- LeRobot 数据 --> Zarr 转换 -----------+
  |                                              v
  |                              diffusion_policy 对应任务配置
  |                                  dataset --> workspace
  |                                              |
  |                                              v
  |                                       训练 checkpoint
  |                                              |
  |                                              +--> 在线推理服务
  |
  +-- LeRobot-Anything-U-Arm
        |
        +-- 主臂机械方案、舵机读取/归零工具
        +-- 可选 ROS 主从遥操作、录制与仿真路径
        `-- 当前 ER3Pro 主线直接读取串口，不必经过其 ROS 节点
```

## 图中接口的归属

- `arm_bridge.cpp` 和 `uarm_er3pro_rt.cpp` 位于 `src/xCoreSDK_cpp-v0.7.1/example/`，承担本工程的硬件集成职责。
- 推理服务入口包括 `src/tidybot2/policy_server.py`、`src/diffusion_policy/policy_server.py` 和 `src/diffusion_policy/policy_server_uarm_er3pro.py`，应与任务配置和 checkpoint 配套选择。
- HDF5 转换入口为 `src/tidybot2/convert_to_robomimic_hdf5.py`；Zarr 转换入口为 `src/diffusion_policy/diffusion_policy/scripts/lerobot_uarm_er3pro_to_zarr.py`。两条路径对应不同的数据适配和任务配置。
- `LeRobot-Anything-U-Arm` 子模块、Python `lerobot` 包和 LeRobot 数据格式是不同概念。
- 图中底盘与夹爪链路对应检查时的 `ros1_udp` 和 `rs485_epg` 配置；底盘后端提供的 `base_pose` 是软件对命令的积分估计，并非 P500 SLAM/里程计反馈。

## 分析时的版本基线

| 仓库 | 本地 checkout commit |
| --- | --- |
| P500_ER3P_ws | `22941b152d76c1af4422ce356f9a7a2ad88443d1` |
| src/tidybot2 | `0ff7068629df351f58fe9b2ac5508ba9db8898c2` |
| src/diffusion_policy | `eca0797fde09abb0a5022e4c7834307a35bccd1a` |
| src/LeRobot-Anything-U-Arm | `321de79bffb94eab8ba006e1390c6f67d609e0b6` |

分析时主仓库位于 `main`，三个子模块均处于 detached HEAD，且与主仓库记录的子模块提交一致。
