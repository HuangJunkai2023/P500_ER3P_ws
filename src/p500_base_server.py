#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P500 底盘 RPC 服务器
参考 tidybot2/base_server.py 架构
允许其他进程通过 RPC 与底盘通信
"""

import time
from multiprocessing.managers import BaseManager as MPBaseManager
from p500_base_controller import P500Vehicle

# RPC 配置
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 50000
RPC_AUTHKEY = b'secret password'


class Base:
    """底盘 RPC 接口类"""
    
    def __init__(self, chassis_ip="192.168.1.100", max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        """
        初始化底盘
        :param chassis_ip: 底盘 IP 地址
        :param max_vel: 最大速度 (vx, vy, omega)
        :param max_accel: 最大加速度
        """
        self.chassis_ip = chassis_ip
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.vehicle = None
    
    def reset(self):
        """重置底盘"""
        # 停止之前的控制
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()
        
        # 创建新的底盘实例
        self.vehicle = P500Vehicle(
            chassis_ip=self.chassis_ip,
            max_vel=self.max_vel,
            max_accel=self.max_accel
        )
        
        # 启动控制循环
        self.vehicle.start_control()
        while not self.vehicle.control_loop_running:
            time.sleep(0.01)
        
        print("[Base RPC] 底盘已重置")
    
    def execute_action(self, action):
        """
        执行动作
        :param action: 字典，包含 'base_pose' 键
        """
        if self.vehicle is not None:
            self.vehicle.set_target_position(action['base_pose'])
    
    def get_state(self):
        """
        获取底盘状态
        :return: 字典，包含 'base_pose' 键
        """
        if self.vehicle is not None:
            state = {'base_pose': self.vehicle.x.copy()}
            return state
        return {'base_pose': [0.0, 0.0, 0.0]}
    
    def close(self):
        """关闭底盘"""
        if self.vehicle is not None:
            if self.vehicle.control_loop_running:
                self.vehicle.stop_control()
        print("[Base RPC] 底盘已关闭")


class BaseManager(MPBaseManager):
    """底盘管理器"""
    pass


# 注册 Base 类到管理器
BaseManager.register('Base', Base)


if __name__ == '__main__':
    print("=" * 60)
    print("启动 P500 底盘 RPC 服务器")
    print("=" * 60)
    print(f"主机: {BASE_RPC_HOST}")
    print(f"端口: {BASE_RPC_PORT}")
    print("=" * 60)
    
    # 创建并启动服务器
    manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    
    print(f"\n✓ 底盘管理服务器已启动")
    print(f"  监听地址: {BASE_RPC_HOST}:{BASE_RPC_PORT}")
    print(f"\n等待客户端连接...\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\n收到停止信号，正在关闭服务器...")
        print("服务器已关闭")


# 客户端测试代码（取消注释以测试）
"""
if __name__ == '__main__':
    import numpy as np
    from constants import POLICY_CONTROL_PERIOD
    
    # 连接到服务器
    manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    
    # 获取底盘代理对象
    base = manager.Base(chassis_ip="192.168.1.100")
    
    try:
        # 重置底盘
        base.reset()
        print("底盘已重置")
        
        # 测试移动
        for i in range(50):
            # 设置目标位姿
            target_x = (i / 50) * 0.5  # 前进 0.5m
            base.execute_action({'base_pose': np.array([target_x, 0.0, 0.0])})
            
            # 获取当前状态
            state = base.get_state()
            print(f"步骤 {i}: {state}")
            
            time.sleep(POLICY_CONTROL_PERIOD)
    
    finally:
        base.close()
        print("测试完成")
"""
