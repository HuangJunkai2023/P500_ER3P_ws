#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
珞石机械臂 RPC 服务器
参考 tidybot2/arm_server.py 架构
允许其他进程通过 RPC 与机械臂通信
"""

import queue
import time
from multiprocessing.managers import BaseManager as MPBaseManager
import numpy as np
from rokae_arm_controller import RokaeArmController, JointCompliantController
from scipy.spatial.transform import Rotation as R

# RPC 配置
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 50001
RPC_AUTHKEY = b'secret password'


class Arm:
    """机械臂 RPC 接口类"""
    
    def __init__(self, arm_ip="192.168.1.160"):
        """
        初始化机械臂
        :param arm_ip: 机械臂控制器 IP 地址
        """
        self.arm_ip = arm_ip
        self.arm = RokaeArmController(arm_ip=arm_ip)
        self.command_queue = queue.Queue(1)
        self.controller = None
    
    def reset(self):
        """重置机械臂"""
        # 停止之前的循环控制
        if self.arm.cyclic_running:
            time.sleep(0.75)  # 等待机械臂停止移动
            self.arm.stop_cyclic()
        
        # 清除故障
        self.arm.clear_faults()
        
        # 回到初始姿态
        self.arm.open_gripper()
        self.arm.retract()
        time.sleep(1)  # 等待回零完成
        
        # 创建新的控制器实例
        self.controller = JointCompliantController(self.command_queue)
        
        # 启动循环控制
        self.arm.init_cyclic(self.controller.control_callback)
        while not self.arm.cyclic_running:
            time.sleep(0.01)
        
        print("[Arm RPC] 机械臂已重置")
    
    def execute_action(self, action):
        """
        执行动作
        :param action: 字典，包含 'arm_pos', 'arm_quat', 'gripper_pos'
        """
        if self.arm is not None:
            # 获取目标位姿
            target_pos = action['arm_pos']
            target_quat = action['arm_quat']
            target_gripper = action['gripper_pos'].item()
            
            # 移动到目标位姿
            self.arm.move_to_pose(target_pos, target_quat)
            
            # 控制夹爪
            self.arm.set_gripper_position(target_gripper)
            
            # 如果使用关节空间控制，可以用队列
            # qpos = self._inverse_kinematics(target_pos, target_quat)
            # self.command_queue.put((qpos, target_gripper))
    
    def get_state(self):
        """
        获取机械臂状态
        :return: 字典，包含 'arm_pos', 'arm_quat', 'gripper_pos'
        """
        if self.arm is not None:
            # 获取末端位姿
            arm_pos, arm_quat = self.arm.get_tool_pose()
            
            # 强制四元数唯一性
            if arm_quat[3] < 0.0:
                np.negative(arm_quat, out=arm_quat)
            
            state = {
                'arm_pos': arm_pos,
                'arm_quat': arm_quat,
                'gripper_pos': np.array([self.arm.gripper_pos]),
            }
            return state
        
        # 默认状态
        return {
            'arm_pos': np.array([0.4, 0.0, 0.3]),
            'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),
            'gripper_pos': np.array([0.0]),
        }
    
    def close(self):
        """关闭机械臂"""
        if self.arm is not None:
            if self.arm.cyclic_running:
                time.sleep(0.75)  # 等待停止
                self.arm.stop_cyclic()
            self.arm.disconnect()
        print("[Arm RPC] 机械臂已关闭")


class ArmManager(MPBaseManager):
    """机械臂管理器"""
    pass


# 注册 Arm 类到管理器
ArmManager.register('Arm', Arm)


if __name__ == '__main__':
    print("=" * 60)
    print("启动珞石机械臂 RPC 服务器")
    print("=" * 60)
    print(f"主机: {ARM_RPC_HOST}")
    print(f"端口: {ARM_RPC_PORT}")
    print("=" * 60)
    
    # 创建并启动服务器
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    
    print(f"\n✓ 机械臂管理服务器已启动")
    print(f"  监听地址: {ARM_RPC_HOST}:{ARM_RPC_PORT}")
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
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    
    # 获取机械臂代理对象
    arm = manager.Arm(arm_ip="192.168.1.160")
    
    try:
        # 重置机械臂
        arm.reset()
        print("机械臂已重置")
        
        # 测试移动
        for i in range(50):
            # 获取当前状态
            state = arm.get_state()
            
            # 设置目标位姿（稍微向上移动）
            target_pos = state['arm_pos'] + np.array([0.0, 0.0, 0.001])
            arm.execute_action({
                'arm_pos': target_pos,
                'arm_quat': state['arm_quat'],
                'gripper_pos': np.array([0.0]),
            })
            
            print(f"步骤 {i}: 位置 {state['arm_pos']}")
            time.sleep(POLICY_CONTROL_PERIOD)
    
    finally:
        arm.close()
        print("测试完成")
"""
