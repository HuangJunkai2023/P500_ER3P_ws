#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P500 底盘控制器
参考 tidybot2/base_controller.py 架构，使用 P500 底盘 API
"""

import os
import sys
import time
import numpy as np

# 添加底盘API路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, 'p500'))
import robot_api as chassis_api


class P500Vehicle:
    """P500 全向底盘控制器"""
    
    def __init__(self, chassis_ip="192.168.1.100", max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        """
        初始化 P500 底盘
        :param chassis_ip: 底盘控制器 IP
        :param max_vel: 最大速度 (vx, vy, omega) in (m/s, m/s, rad/s)
        :param max_accel: 最大加速度 (ax, ay, alpha) in (m/s^2, m/s^2, rad/s^2)
        """
        self.chassis_ip = chassis_ip
        self.max_vel = np.array(max_vel)
        self.max_accel = np.array(max_accel)
        
        # 当前状态
        self.x = np.array([0.0, 0.0, 0.0])  # [x, y, theta] 位姿
        self.x_dot = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega] 速度
        
        # 目标位姿
        self.target_x = np.array([0.0, 0.0, 0.0])
        
        # 控制循环状态
        self.control_loop_running = False
        
        # 连接底盘
        self._connect()
    
    def _connect(self):
        """连接到底盘"""
        print(f"[P500 底盘] 连接到 {self.chassis_ip}...")
        chassis_api.setHost(f"http://{self.chassis_ip}:8080")
        
        # 登录
        res = chassis_api.login('admin', 'admin')
        if not res:
            raise Exception(f"底盘登录失败: {res.msg if hasattr(res, 'msg') else 'Unknown error'}")
        
        print("[P500 底盘] 登录成功")
        
        # 获取初始状态
        self._update_state()
        print(f"[P500 底盘] 初始位姿: {self.x}")
    
    def _update_state(self):
        """从底盘获取当前状态"""
        try:
            res = chassis_api.robotStatus()
            if res and res.data:
                pose = res.data.get('pose', {})
                twist = res.data.get('twist', {})
                
                position = pose.get('position', {})
                orientation = pose.get('orientation', {})
                linear = twist.get('linear', {})
                angular = twist.get('angular', {})
                
                # 更新位姿
                self.x[0] = position.get('x', 0.0)
                self.x[1] = position.get('y', 0.0)
                self.x[2] = orientation.get('z', 0.0)  # theta
                
                # 更新速度
                self.x_dot[0] = linear.get('x', 0.0)
                self.x_dot[1] = linear.get('y', 0.0)
                self.x_dot[2] = angular.get('z', 0.0)
                
        except Exception as e:
            print(f"[P500 底盘] 获取状态失败: {e}")
    
    def start_control(self):
        """启动控制循环（P500 API 自动处理）"""
        self.control_loop_running = True
        print("[P500 底盘] 控制循环已启动")
    
    def stop_control(self):
        """停止控制循环"""
        # 停止底盘运动
        try:
            chassis_api.setSpeed(0.0, 0.0, 0.0)
            print("[P500 底盘] 已停止运动")
        except Exception as e:
            print(f"[P500 底盘] 停止失败: {e}")
        
        self.control_loop_running = False
    
    def set_target_position(self, target_pose):
        """
        设置目标位姿
        :param target_pose: [x, y, theta] 目标位姿
        """
        self.target_x = np.array(target_pose)
        
        # 计算位姿误差
        error = self.target_x - self.x
        
        # 角度归一化到 [-pi, pi]
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        
        # 简单 P 控制器
        Kp = np.array([1.0, 1.0, 1.5])  # 位置和角度的增益
        
        # 计算期望速度
        desired_vel = Kp * error
        
        # 速度限制
        desired_vel = np.clip(desired_vel, -self.max_vel, self.max_vel)
        
        # 发送速度指令到底盘
        try:
            res = chassis_api.setSpeed(
                float(desired_vel[0]),
                float(desired_vel[1]),
                float(desired_vel[2])
            )
            
            if res:
                # 更新状态（如果 API 不返回实时状态，使用估计值）
                self._update_state()
        
        except Exception as e:
            print(f"[P500 底盘] 设置速度失败: {e}")


if __name__ == '__main__':
    """测试底盘控制器"""
    import time
    
    print("=" * 60)
    print("P500 底盘控制器测试")
    print("=" * 60)
    
    # 创建底盘对象
    vehicle = P500Vehicle(
        chassis_ip="192.168.1.100",
        max_vel=(0.3, 0.3, 0.5),
        max_accel=(0.2, 0.2, 0.3)
    )
    
    try:
        # 启动控制
        vehicle.start_control()
        
        print("\n测试 1: 前进 0.2m")
        vehicle.set_target_position([0.2, 0.0, 0.0])
        time.sleep(2)
        print(f"当前位姿: {vehicle.x}")
        
        print("\n测试 2: 左移 0.2m")
        vehicle.set_target_position([0.2, 0.2, 0.0])
        time.sleep(2)
        print(f"当前位姿: {vehicle.x}")
        
        print("\n测试 3: 旋转 90 度")
        vehicle.set_target_position([0.2, 0.2, np.pi/2])
        time.sleep(2)
        print(f"当前位姿: {vehicle.x}")
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 停止控制
        vehicle.stop_control()
        print("\n底盘已停止")
