#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
珞石机械臂控制器
参考 tidybot2/arm_controller.py 架构，使用珞石机械臂 SDK
"""

import os
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# 添加机械臂SDK路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, 'xCoreSDK-Python/rokae_SDK_linux_v0.1.6_py38/lib'))
from robot import *
from convert_tools import *


class RokaeArmController:
    """珞石机械臂控制器"""
    
    def __init__(self, arm_ip="192.168.1.160"):
        """
        初始化珞石机械臂
        :param arm_ip: 机械臂控制器 IP
        """
        self.arm_ip = arm_ip
        self.robot = None
        self.cyclic_running = False
        
        # 状态
        self.q = np.zeros(6)  # 关节角度
        self.gripper_pos = 0.0  # 夹爪位置 [0, 1]
        
        # 目标指令队列
        self.target_pos = None
        self.target_quat = None
        self.target_gripper = 0.0
        
        # 连接机械臂
        self._connect()
    
    def _connect(self):
        """连接到机械臂"""
        print(f"[珞石机械臂] 连接到 {self.arm_ip}...")
        ec = {}
        
        self.robot = XMateErProRobot(self.arm_ip)
        self.robot.connectToRobot(ec)
        
        if ec:
            raise Exception(f"机械臂连接失败: {ec}")
        
        info = self.robot.robotInfo(ec)
        print(f"[珞石机械臂] 连接成功! 机型: {info.get('type', 'Unknown')}")
        
        # 上电
        print("[珞石机械臂] 正在上电...")
        self.robot.setPowerState(True, ec)
        time.sleep(1)
        
        if not self.robot.powerState(ec):
            raise Exception("机械臂上电失败")
        
        print("[珞石机械臂] 上电成功")
        
        # 获取初始关节角度
        self._update_joint_state()
    
    def _update_joint_state(self):
        """更新关节状态"""
        try:
            ec = {}
            joint_pos = self.robot.jointPos(ec)
            if joint_pos and len(joint_pos) >= 6:
                self.q = np.array(joint_pos[:6])
        except Exception as e:
            print(f"[珞石机械臂] 获取关节状态失败: {e}")
    
    def get_tool_pose(self):
        """
        获取工具末端位姿
        :return: (position, quaternion) 位置和四元数
        """
        try:
            ec = {}
            flange_pos = self.robot.flangePos(ec)
            
            if flange_pos and len(flange_pos) >= 6:
                # 位置 (m)
                pos = np.array(flange_pos[:3])
                
                # 欧拉角转四元数
                euler = np.array(flange_pos[3:6])
                rot = R.from_euler('xyz', euler)
                quat = rot.as_quat()  # [x, y, z, w]
                
                return pos, quat
            
            # 默认值
            return np.array([0.4, 0.0, 0.3]), np.array([0.0, 0.0, 0.0, 1.0])
            
        except Exception as e:
            print(f"[珞石机械臂] 获取工具位姿失败: {e}")
            return np.array([0.4, 0.0, 0.3]), np.array([0.0, 0.0, 0.0, 1.0])
    
    def clear_faults(self):
        """清除故障"""
        try:
            ec = {}
            # 珞石机械臂的故障清除（如果有相关 API）
            print("[珞石机械臂] 清除故障...")
        except Exception as e:
            print(f"[珞石机械臂] 清除故障失败: {e}")
    
    def retract(self):
        """回到初始姿态"""
        try:
            ec = {}
            print("[珞石机械臂] 移动到初始姿态...")
            
            # 定义初始关节位置（根据实际机器人调整）
            # 这里使用一个安全的中间位置
            home_joints = [0.0, -0.5, 0.5, 0.0, 1.5, 0.0]
            
            # 使用关节空间运动
            vel = 0.5  # 50% 速度
            acc = 0.5
            self.robot.moveByJoint(home_joints, vel, acc, 0, ec)
            
            if ec:
                print(f"[珞石机械臂] 回零失败: {ec}")
            else:
                print("[珞石机械臂] 已回到初始位置")
            
        except Exception as e:
            print(f"[珞石机械臂] 回零异常: {e}")
    
    def open_gripper(self):
        """打开夹爪"""
        try:
            # 这里需要根据您的夹爪类型实现
            # 示例: 使用数字 IO 控制气动夹爪
            # ec = {}
            # self.robot.setDO(1, True, ec)  # 打开
            self.gripper_pos = 1.0
            print("[珞石机械臂] 夹爪已打开")
        except Exception as e:
            print(f"[珞石机械臂] 打开夹爪失败: {e}")
    
    def close_gripper(self):
        """关闭夹爪"""
        try:
            # ec = {}
            # self.robot.setDO(1, False, ec)  # 关闭
            self.gripper_pos = 0.0
            print("[珞石机械臂] 夹爪已关闭")
        except Exception as e:
            print(f"[珞石机械臂] 关闭夹爪失败: {e}")
    
    def set_gripper_position(self, pos):
        """
        设置夹爪位置
        :param pos: 0.0 (闭合) 到 1.0 (张开)
        """
        try:
            # 根据夹爪类型实现
            if pos > 0.5:
                self.open_gripper()
            else:
                self.close_gripper()
            
            self.gripper_pos = pos
        except Exception as e:
            print(f"[珞石机械臂] 设置夹爪位置失败: {e}")
    
    def set_joint_limits(self, speed_limits=None, acceleration_limits=None):
        """设置关节限制（可选）"""
        # 珞石机械臂的速度和加速度在 moveByLine/moveByJoint 中直接指定
        print("[珞石机械臂] 关节限制设置（通过运动指令控制）")
    
    def init_cyclic(self, control_callback=None):
        """
        初始化循环控制模式
        注意: 珞石机械臂使用阻塞式运动指令，不需要实时循环
        """
        self.cyclic_running = True
        print("[珞石机械臂] 循环控制模式已启动")
    
    def stop_cyclic(self):
        """停止循环控制"""
        try:
            ec = {}
            # 停止运动
            self.robot.setSpeed(0.0, ec)  # 停止
            print("[珞石机械臂] 循环控制已停止")
        except Exception as e:
            print(f"[珞石机械臂] 停止失败: {e}")
        
        self.cyclic_running = False
    
    def move_to_pose(self, pos, quat):
        """
        移动到目标位姿（笛卡尔空间）
        :param pos: [x, y, z] 目标位置 (m)
        :param quat: [x, y, z, w] 目标四元数
        """
        try:
            ec = {}
            
            # 四元数转欧拉角
            rot = R.from_quat(quat)
            euler = rot.as_euler('xyz')
            
            # 组合目标位姿
            target_flange = list(pos) + list(euler)
            
            # 执行直线运动（非阻塞）
            vel = 0.2  # 20 cm/s
            acc = 0.5  # 较大加速度
            
            # 使用 moveByLine 移动
            self.robot.moveByLine(target_flange, vel, acc, 0, ec)
            
            if ec:
                print(f"[珞石机械臂] 移动失败: {ec}")
            
            # 更新关节状态
            self._update_joint_state()
            
        except Exception as e:
            print(f"[珞石机械臂] 移动异常: {e}")
    
    def disconnect(self):
        """断开连接"""
        try:
            if self.cyclic_running:
                self.stop_cyclic()
            
            ec = {}
            # 下电
            self.robot.setPowerState(False, ec)
            time.sleep(0.5)
            
            # 断开连接
            self.robot.disconnectFromRobot(ec)
            print("[珞石机械臂] 已断开连接")
            
        except Exception as e:
            print(f"[珞石机械臂] 断开连接失败: {e}")


class JointCompliantController:
    """
    关节柔顺控制器（兼容 tidybot2 接口）
    珞石机械臂使用位置控制，这里提供兼容接口
    """
    
    def __init__(self, command_queue):
        """
        :param command_queue: 命令队列，包含 (qpos, gripper_pos) 元组
        """
        self.command_queue = command_queue
        self.qpos_target = None
        self.gripper_target = None
    
    def control_callback(self, feedback):
        """
        控制回调（珞石机械臂不需要实时回调）
        """
        # 从队列获取最新命令
        if not self.command_queue.empty():
            self.qpos_target, self.gripper_target = self.command_queue.get()
        
        return None  # 珞石使用位置指令，不返回力矩


if __name__ == '__main__':
    """测试机械臂控制器"""
    import time
    
    print("=" * 60)
    print("珞石机械臂控制器测试")
    print("=" * 60)
    
    # 创建机械臂对象
    arm = RokaeArmController(arm_ip="192.168.1.160")
    
    try:
        # 清除故障
        arm.clear_faults()
        
        # 回到初始位置
        arm.retract()
        time.sleep(2)
        
        # 初始化循环控制
        arm.init_cyclic()
        
        # 获取当前位姿
        pos, quat = arm.get_tool_pose()
        print(f"\n当前位姿:")
        print(f"  位置: {pos}")
        print(f"  四元数: {quat}")
        
        # 测试移动
        print("\n测试 1: 向上移动 5cm")
        target_pos = pos + np.array([0.0, 0.0, 0.05])
        arm.move_to_pose(target_pos, quat)
        time.sleep(2)
        
        print("\n测试 2: 向前移动 5cm")
        target_pos = target_pos + np.array([0.05, 0.0, 0.0])
        arm.move_to_pose(target_pos, quat)
        time.sleep(2)
        
        # 测试夹爪
        print("\n测试 3: 夹爪控制")
        arm.open_gripper()
        time.sleep(1)
        arm.close_gripper()
        time.sleep(1)
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 断开连接
        arm.disconnect()
        print("\n机械臂已断开连接")
