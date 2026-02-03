# P500 + 珞石机械臂手机示教系统

完全参考 **tidybot2** 的架构，只替换了底层的机械臂和底盘 SDK。

## 🏗️ 架构说明

```
tidybot2 原架构:
├── base_controller.py    → 底盘底层控制器 (Phoenix 6 电机)
├── arm_controller.py     → 机械臂控制器 (Kinova Gen3)
├── base_server.py        → 底盘 RPC 服务器
├── arm_server.py         → 机械臂 RPC 服务器
├── real_env.py           → 真实环境 (通过 RPC 连接)
├── policies.py           → 示教策略 (WebXR + Flask)
└── main.py               → 主程序

P500 适配版本:
├── p500_base_controller.py    → P500 底盘控制器 ✨新建
├── rokae_arm_controller.py    → 珞石机械臂控制器 ✨新建
├── p500_base_server.py        → P500 底盘 RPC 服务器 ✨新建
├── rokae_arm_server.py        → 珞石机械臂 RPC 服务器 ✨新建
├── p500_real_env.py           → 真实环境 (连接上述服务器) ✨新建
├── tidybot2/policies.py       → 示教策略 (原封不动) ✅复用
└── p500_teleop_real.py        → 主程序 ✨新建
```

## 🚀 使用流程

### 1. 启动 RPC 服务器

**打开 3 个终端**，分别运行：

```bash
# 终端 1: 启动底盘 RPC 服务器
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src
python p500_base_server.py
# 等待显示: Base manager server started at localhost:50000

# 终端 2: 启动机械臂 RPC 服务器
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src
python rokae_arm_server.py
# 等待显示: Arm manager server started at localhost:50001

# 终端 3: 启动主程序
cd /home/huaxi/scu_robotics/P500_ER3P_ws/src
python p500_teleop_real.py --teleop
```

### 2. 连接手机

1. 手机连接与机器人相同的 WiFi
2. 在手机浏览器打开终端 3 显示的地址（如 `http://192.168.1.50:5000`）
3. 允许浏览器使用摄像头和传感器
4. 点击 **"Start episode"** 开始示教

### 3. 操作方式

与 tidybot2 完全相同：
- 📱 **触摸左侧 (90%)**: 控制机械臂（移动手机=移动末端，滑动=控制夹爪）
- 📱 **触摸右侧 (10%)**: 控制底盘（平移+旋转）
- 🔴 **屏幕变红**: 底盘控制模式
- 🔵 **屏幕变蓝**: 机械臂控制模式

## 📁 文件清单

| 文件 | 说明 | 状态 |
|------|------|------|
| `p500_base_controller.py` | P500 底盘控制器，替换原始的 Phoenix 6 电机控制 | ✨新建 |
| `rokae_arm_controller.py` | 珞石机械臂控制器，替换原始的 Kinova 控制 | ✨新建 |
| `p500_base_server.py` | 底盘 RPC 服务器，接口与 tidybot2 一致 | ✨新建 |
| `rokae_arm_server.py` | 机械臂 RPC 服务器，接口与 tidybot2 一致 | ✨新建 |
| `p500_real_env.py` | 真实环境类，完全兼容 tidybot2 的 `RealEnv` | ✨新建 |
| `p500_teleop_real.py` | 主程序，基于 tidybot2 的 `main.py` | ✨新建 |
| `tidybot2/policies.py` | 示教策略（WebXR + Flask）| ✅直接复用 |
| `tidybot2/templates/index.html` | 手机 Web 界面 | ✅直接复用 |
| `tidybot2/episode_storage.py` | 数据存储 | ✅直接复用 |

## 💡 为什么使用 RPC 架构？

参考 tidybot2 的设计理念：

1. **隔离实时控制**: 底盘和机械臂的底层控制运行在独立进程中，避免主程序的 Python GIL 影响实时性
2. **防止延迟尖峰**: 非时间敏感的操作（如数据记录、网络通信）在单独进程，不干扰控制循环
3. **安全性**: 如果主程序崩溃，RPC 服务器可以安全停止机器人
4. **模块化**: 每个组件独立开发和测试

## 🔧 配置选项

### IP 地址配置

在 `p500_teleop_real.py` 中：

```python
--arm-ip 192.168.1.160      # 机械臂 IP
--chassis-ip 192.168.1.100  # 底盘 IP
```

或在 `p500_base_server.py` 和 `rokae_arm_server.py` 中修改默认值。

### 速度和加速度限制

在 `p500_base_controller.py`:

```python
max_vel=(0.5, 0.5, 1.57)      # m/s, m/s, rad/s
max_accel=(0.25, 0.25, 0.79)  # m/s^2, m/s^2, rad/s^2
```

### 控制增益

在 `p500_base_controller.py` 的 `set_target_position`:

```python
Kp = np.array([1.0, 1.0, 1.5])  # 位置和角度增益
```

## ⚙️ 命令参数

```bash
python p500_teleop_real.py [选项]

选项:
  --teleop              启用手机示教模式
  --arm-ip IP           机械臂 IP (默认: 192.168.1.160)
  --chassis-ip IP       底盘 IP (默认: 192.168.1.100)
  --save                保存示教数据
  --output-dir DIR      数据保存目录 (默认: data/demos)
  --ssl                 使用 HTTPS (某些设备需要)
```

## 🧪 测试

### 测试底盘控制器
```bash
python p500_base_controller.py
```

### 测试机械臂控制器
```bash
python rokae_arm_controller.py
```

### 测试环境
```bash
# 先启动两个 RPC 服务器，然后:
python p500_real_env.py
```

## 📊 与原版的对比

| 组件 | tidybot2 原版 | P500 适配版 | 变化 |
|------|---------------|-------------|------|
| 底盘硬件 | 自制全向底盘 (Phoenix 6) | P500 商用底盘 | ✨替换 |
| 机械臂 | Kinova Gen3 | 珞石 xMate ER Pro | ✨替换 |
| 底盘控制 | 实时电机力矩控制 | HTTP API 速度控制 | ✨替换 |
| 机械臂控制 | 实时力矩控制 | 位置控制 | ✨替换 |
| RPC 架构 | multiprocessing.Manager | 相同 | ✅保持 |
| 示教界面 | WebXR + Flask | 相同 | ✅保持 |
| 数据记录 | HDF5 | 相同 | ✅保持 |
| 主程序逻辑 | main.py | 相同 | ✅保持 |

## ⚠️ 注意事项

1. **RPC 服务器必须先启动**: 主程序会尝试连接 `localhost:50000` 和 `localhost:50001`
2. **网络连接**: 确保机械臂和底盘可访问
3. **安全距离**: 首次使用时保持安全距离，准备急停
4. **控制延迟**: P500 底盘使用 HTTP API，可能比 tidybot2 的直接电机控制延迟稍高
5. **机械臂速度**: 珞石机械臂使用位置控制，运动可能比力矩控制稍慢

## 🎯 与之前版本的区别

之前创建的 `p500_teleop_env.py` 和 `p500_teleop_main.py` 是**单进程版本**，直接在主程序中控制硬件。

新版本使用 **RPC 服务器架构**，完全按照 tidybot2 的设计：
- ✅ 更安全（独立进程）
- ✅ 更模块化（可以分别测试）
- ✅ 更符合原工程架构
- ✅ 可以无缝使用 tidybot2 的所有功能（数据记录、策略训练等）

## 🔗 参考

- [TidyBot++ 论文](https://arxiv.org/abs/2412.10447)
- [TidyBot++ GitHub](http://tidybot2.github.io)
- [WebXR API](https://www.w3.org/TR/webxr/)

---

**开始使用吧！🎉**
