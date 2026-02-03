#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一控制机械臂和底盘，并实时反馈状态
Unified Robot System Control - Arm + Chassis
"""

import os
import sys
import time
import threading
from typing import Dict, Any

# 获取当前脚本所在目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 添加机械臂SDK路径 (Linux版本)
sys.path.append(os.path.join(SCRIPT_DIR, 'xCoreSDK-Python/rokae_SDK_linux_v0.1.6_py38/lib'))
from robot import *
from convert_tools import *

# 添加底盘API路径
sys.path.append(os.path.join(SCRIPT_DIR, 'p500'))
import robot_api as chassis_api


class UnifiedRobotSystem:
    """统一的机器人系统控制类"""
    
    def __init__(self, arm_ip="192.168.1.160", chassis_ip="192.168.1.100"):
        """
        初始化机器人系统
        :param arm_ip: 机械臂控制器IP地址
        :param chassis_ip: 底盘控制器IP地址
        """
        self.arm_ip = arm_ip
        self.chassis_ip = chassis_ip
        
        # 机械臂对象
        self.arm_robot = None
        self.arm_connected = False
        self.arm_enabled = False
        
        # 底盘状态
        self.chassis_connected = False
        self.chassis_status = {}
        
        # 状态反馈标志
        self.running = False
        self.feedback_threads = []
        
    def connect_arm(self):
        """连接机械臂"""
        try:
            print(f"[机械臂] 正在连接到 {self.arm_ip}...")
            ec = {}
            self.arm_robot = XMateErProRobot(self.arm_ip)
            self.arm_robot.connectToRobot(ec)
            
            if ec:
                print(f"[机械臂] 连接失败: {ec}")
                return False
                
            # 获取机器人信息
            info = self.arm_robot.robotInfo(ec)
            print(f"[机械臂] 连接成功!")
            print(f"[机械臂] 机型: {info.get('type', 'Unknown')}, "
                  f"轴数: {info.get('joint_num', 0)}, "
                  f"版本: {info.get('version', 'Unknown')}")
            
            self.arm_connected = True
            return True
            
        except Exception as e:
            print(f"[机械臂] 连接异常: {e}")
            return False
    
    def enable_arm(self):
        """使能机械臂（上电）"""
        if not self.arm_connected:
            print("[机械臂] 未连接，无法使能")
            return False
            
        try:
            ec = {}
            print("[机械臂] 正在上电...")
            self.arm_robot.setPowerState(True, ec)
            time.sleep(1)
            
            power = self.arm_robot.powerState(ec)
            if power:
                print("[机械臂] 上电成功")
                self.arm_enabled = True
                return True
            else:
                print("[机械臂] 上电失败")
                return False
                
        except Exception as e:
            print(f"[机械臂] 上电异常: {e}")
            return False
    
    def connect_chassis(self):
        """连接底盘"""
        try:
            print(f"[底盘] 正在连接到 {self.chassis_ip}...")
            chassis_api.setHost(f"http://{self.chassis_ip}:8080")
            
            # 登录
            res = chassis_api.login('admin', 'admin')
            if not res:
                print(f"[底盘] 登录失败: {res.msg}")
                return False
            
            print("[底盘] 登录成功")
            
            # 获取状态
            res = chassis_api.robotStatus()
            if res:
                print(f"[底盘] 连接成功! 状态: {res.data.get('state', 'Unknown')}")
                self.chassis_connected = True
                return True
            else:
                print(f"[底盘] 获取状态失败: {res.msg}")
                return False
                
        except Exception as e:
            print(f"[底盘] 连接异常: {e}")
            return False
    
    def enable_chassis(self):
        """使能底盘"""
        if not self.chassis_connected:
            print("[底盘] 未连接，无法使能")
            return False
            
        try:
            res = chassis_api.robotStatus()
            if res:
                print(f"[底盘] 使能成功，当前状态: {res.data.get('msg', 'Ready')}")
                return True
            else:
                print(f"[底盘] 使能失败: {res.msg}")
                return False
                
        except Exception as e:
            print(f"[底盘] 使能异常: {e}")
            return False
    
    def get_arm_end_pose(self) -> Dict[str, Any]:
        """
        获取机械臂末端位姿 (x, y, z, rx, ry, rz)
        :return: {'x': float, 'y': float, 'z': float, 'rx': float, 'ry': float, 'rz': float}
        """
        if not self.arm_connected or not self.arm_enabled:
            return {}
        
        try:
            ec = {}
            flange_pos = self.arm_robot.flangePos(ec)
            
            if flange_pos and len(flange_pos) >= 6:
                return {
                    'x': flange_pos[0],
                    'y': flange_pos[1],
                    'z': flange_pos[2],
                    'rx': flange_pos[3],
                    'ry': flange_pos[4],
                    'rz': flange_pos[5]
                }
            return {}
            
        except Exception as e:
            print(f"[机械臂] 获取末端位姿异常: {e}")
            return {}
    
    def get_chassis_velocity(self) -> Dict[str, Any]:
        """
        获取底盘速度 (v: 线速度, w: 角速度)
        :return: {'v': float, 'w': float, 'x': float, 'y': float, 'theta': float}
        """
        if not self.chassis_connected:
            return {}
        
        try:
            res = chassis_api.robotStatus()
            if res and res.data:
                twist = res.data.get('twist', {})
                pose = res.data.get('pose', {})
                position = pose.get('position', {})
                orientation = pose.get('orientation', {})
                
                return {
                    'v': twist.get('linear', {}).get('x', 0.0),  # 线速度
                    'w': twist.get('angular', {}).get('z', 0.0),  # 角速度
                    'x': position.get('x', 0.0),
                    'y': position.get('y', 0.0),
                    'theta': orientation.get('z', 0.0),
                    'state': res.data.get('state', -1),
                    'msg': res.data.get('msg', '')
                }
            return {}
            
        except Exception as e:
            print(f"[底盘] 获取速度信息异常: {e}")
            return {}
    
    def arm_feedback_loop(self):
        """机械臂状态反馈循环"""
        print("[机械臂] 启动状态反馈线程...")
        while self.running:
            try:
                pose = self.get_arm_end_pose()
                if pose:
                    print(f"[机械臂] 末端位姿: "
                          f"X={pose['x']:.4f}, Y={pose['y']:.4f}, Z={pose['z']:.4f}, "
                          f"RX={pose['rx']:.4f}, RY={pose['ry']:.4f}, RZ={pose['rz']:.4f}")
                time.sleep(0.5)  # 2Hz 反馈频率
                
            except Exception as e:
                print(f"[机械臂] 反馈异常: {e}")
                time.sleep(1)
    
    def chassis_feedback_loop(self):
        """底盘状态反馈循环"""
        print("[底盘] 启动状态反馈线程...")
        while self.running:
            try:
                status = self.get_chassis_velocity()
                if status:
                    print(f"[底盘] 速度: V={status['v']:.3f} m/s, W={status['w']:.3f} rad/s | "
                          f"位置: X={status.get('x', 0):.2f}, Y={status.get('y', 0):.2f}, "
                          f"Theta={status.get('theta', 0):.2f} | 状态: {status.get('msg', 'Unknown')}")
                time.sleep(0.5)  # 2Hz 反馈频率
                
            except Exception as e:
                print(f"[底盘] 反馈异常: {e}")
                time.sleep(1)
    
    def start_feedback(self):
        """启动状态反馈"""
        if self.running:
            print("[系统] 反馈线程已在运行")
            return
        
        self.running = True
        
        # 启动机械臂反馈线程
        if self.arm_connected and self.arm_enabled:
            arm_thread = threading.Thread(target=self.arm_feedback_loop, daemon=True)
            arm_thread.start()
            self.feedback_threads.append(arm_thread)
        
        # 启动底盘反馈线程
        if self.chassis_connected:
            chassis_thread = threading.Thread(target=self.chassis_feedback_loop, daemon=True)
            chassis_thread.start()
            self.feedback_threads.append(chassis_thread)
        
        print("[系统] 状态反馈已启动")
    
    def stop_feedback(self):
        """停止状态反馈"""
        self.running = False
        print("[系统] 正在停止状态反馈...")
        time.sleep(1)  # 等待线程结束
        self.feedback_threads.clear()
        print("[系统] 状态反馈已停止")
    
    def disconnect(self):
        """断开所有连接"""
        self.stop_feedback()
        
        if self.arm_connected:
            try:
                ec = {}
                self.arm_robot.setPowerState(False, ec)  # 下电
                self.arm_robot.disconnectFromRobot(ec)
                print("[机械臂] 已断开连接")
            except Exception as e:
                print(f"[机械臂] 断开连接异常: {e}")
        
        print("[系统] 所有设备已断开")


def main():
    """主函数"""
    print("=" * 60)
    print("统一机器人控制系统 - 机械臂 + 底盘")
    print("=" * 60)
    
    # 创建机器人系统
    robot_system = UnifiedRobotSystem(
        arm_ip="192.168.1.160",      # 修改为实际的机械臂IP
        chassis_ip="192.168.1.100"   # 修改为实际的底盘IP
    )
    
    try:
        # 1. 连接设备
        print("\n[步骤1] 连接设备...")
        arm_ok = robot_system.connect_arm()
        chassis_ok = robot_system.connect_chassis()
        
        if not arm_ok and not chassis_ok:
            print("[错误] 所有设备连接失败，退出程序")
            return
        
        # 2. 使能设备
        print("\n[步骤2] 使能设备...")
        if arm_ok:
            robot_system.enable_arm()
        if chassis_ok:
            robot_system.enable_chassis()
        
        # 3. 启动状态反馈
        print("\n[步骤3] 启动状态反馈...")
        robot_system.start_feedback()
        
        # 4. 保持运行，持续反馈状态
        print("\n[运行中] 按 Ctrl+C 停止...\n")
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n[系统] 收到停止信号...")
    
    except Exception as e:
        print(f"\n[错误] 程序异常: {e}")
    
    finally:
        # 5. 清理资源
        print("\n[步骤4] 清理资源...")
        robot_system.disconnect()
        print("\n程序退出")


if __name__ == "__main__":
    main()
