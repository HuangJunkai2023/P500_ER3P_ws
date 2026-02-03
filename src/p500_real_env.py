#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P500 + 珞石机械臂真实环境
完全参考 tidybot2/real_env.py 的架构，通过 RPC 连接底盘和机械臂服务器
"""

import sys
import os

# 添加 tidybot2 路径（用于导入相机模块，如果需要）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, 'tidybot2'))

import numpy as np
from p500_base_server import BaseManager, BASE_RPC_HOST, BASE_RPC_PORT, RPC_AUTHKEY
from rokae_arm_server import ArmManager, ARM_RPC_HOST, ARM_RPC_PORT


class P500RealEnv:
    """
    P500 + 珞石机械臂真实环境
    完全兼容 tidybot2 的 RealEnv 接口
    """
    
    def __init__(self, chassis_ip="192.168.1.100", arm_ip="192.168.1.160"):
        """
        初始化环境
        :param chassis_ip: 底盘 IP
        :param arm_ip: 机械臂 IP
        """
        print("=" * 60)
        print("初始化 P500 + 珞石机械臂真实环境")
        print("=" * 60)
        
        # 连接底盘 RPC 服务器
        print(f"\n[1/2] 连接底盘 RPC 服务器 ({BASE_RPC_HOST}:{BASE_RPC_PORT})...")
        base_manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            base_manager.connect()
            print("  ✓ 底盘 RPC 连接成功")
        except ConnectionRefusedError as e:
            raise Exception(
                '无法连接到底盘 RPC 服务器！\n'
                '请先运行: python p500_base_server.py'
            ) from e
        
        # 连接机械臂 RPC 服务器
        print(f"\n[2/2] 连接机械臂 RPC 服务器 ({ARM_RPC_HOST}:{ARM_RPC_PORT})...")
        arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            arm_manager.connect()
            print("  ✓ 机械臂 RPC 连接成功")
        except ConnectionRefusedError as e:
            raise Exception(
                '无法连接到机械臂 RPC 服务器！\n'
                '请先运行: python rokae_arm_server.py'
            ) from e
        
        # 创建 RPC 代理对象
        print(f"\n[3/3] 创建设备代理对象...")
        self.base = base_manager.Base(
            chassis_ip=chassis_ip,
            max_vel=(0.5, 0.5, 1.57),
            max_accel=(0.5, 0.5, 1.57)
        )
        self.arm = arm_manager.Arm(arm_ip=arm_ip)
        print("  ✓ 代理对象创建成功")
        
        # 相机（暂时使用虚拟相机）
        # 如果需要真实相机，取消注释并导入 cameras 模块
        # from cameras import LogitechCamera, KinovaCamera
        # self.base_camera = LogitechCamera(BASE_CAMERA_SERIAL)
        # self.wrist_camera = KinovaCamera()
        self.base_camera = None
        self.wrist_camera = None
        
        print("\n" + "=" * 60)
        print("环境初始化完成！")
        print("=" * 60 + "\n")
    
    def get_obs(self):
        """
        获取观测
        :return: 字典包含 base_pose, arm_pos, arm_quat, gripper_pos, base_image, wrist_image
        """
        obs = {}
        
        # 底盘状态
        obs.update(self.base.get_state())
        
        # 机械臂状态
        obs.update(self.arm.get_state())
        
        # 相机图像
        if self.base_camera is not None:
            obs['base_image'] = self.base_camera.get_image()
        else:
            obs['base_image'] = np.zeros((480, 640, 3), dtype=np.uint8)
        
        if self.wrist_camera is not None:
            obs['wrist_image'] = self.wrist_camera.get_image()
        else:
            obs['wrist_image'] = np.zeros((480, 640, 3), dtype=np.uint8)
        
        return obs
    
    def reset(self):
        """重置环境"""
        print("\n" + "=" * 60)
        print("重置机器人...")
        print("=" * 60)
        
        print('\n[1/2] 重置底盘...')
        self.base.reset()
        print('  ✓ 底盘重置完成')
        
        print('\n[2/2] 重置机械臂...')
        self.arm.reset()
        print('  ✓ 机械臂重置完成')
        
        print("\n" + "=" * 60)
        print("机器人已重置")
        print("=" * 60 + "\n")
    
    def step(self, action):
        """
        执行动作
        :param action: 字典包含 base_pose, arm_pos, arm_quat, gripper_pos
        注意: 不返回 obs，避免使用过时数据
        """
        # 非阻塞执行
        self.base.execute_action(action)
        self.arm.execute_action(action)
    
    def close(self):
        """关闭环境"""
        print("\n" + "=" * 60)
        print("关闭环境...")
        print("=" * 60)
        
        self.base.close()
        self.arm.close()
        
        if self.base_camera is not None:
            self.base_camera.close()
        if self.wrist_camera is not None:
            self.wrist_camera.close()
        
        print("环境已关闭\n")


if __name__ == '__main__':
    """测试环境"""
    import time
    import numpy as np
    
    # 控制周期
    POLICY_CONTROL_PERIOD = 0.1
    
    # 创建环境
    env = P500RealEnv(
        chassis_ip="192.168.1.100",
        arm_ip="192.168.1.160"
    )
    
    try:
        # 持续运行测试
        while True:
            # 重置
            env.reset()
            
            # 运行 100 步
            for i in range(100):
                # 获取观测
                obs = env.get_obs()
                
                # 显示状态
                print(f"[步骤 {i}] "
                      f"底盘: {obs['base_pose']}, "
                      f"机械臂: {obs['arm_pos']}, "
                      f"夹爪: {obs['gripper_pos']}")
                
                # 生成随机动作（小幅度）
                action = {
                    'base_pose': obs['base_pose'] + 0.01 * np.random.rand(3) - 0.005,
                    'arm_pos': obs['arm_pos'] + 0.01 * np.random.rand(3) - 0.005,
                    'arm_quat': obs['arm_quat'],
                    'gripper_pos': np.clip(obs['gripper_pos'] + 0.1 * np.random.rand(1) - 0.05, 0, 1),
                }
                
                # 执行动作
                env.step(action)
                
                # 控制频率
                time.sleep(POLICY_CONTROL_PERIOD)
    
    except KeyboardInterrupt:
        print("\n\n收到停止信号...")
    
    finally:
        env.close()
