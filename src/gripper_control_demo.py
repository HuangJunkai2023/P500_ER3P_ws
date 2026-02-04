#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jodell 夹爪控制示例
使用 JodellTool SDK 控制各种型号的夹爪
"""

from jodellSdk.jodellSdkDemo import *
import time


def test_gripper_basic():
    """基础夹爪控制测试"""
    print("=" * 60)
    print("Jodell 夹爪控制测试")
    print("=" * 60)
    
    # 根据您的夹爪型号选择合适的控制类
    # 可用的夹爪类型:
    # - ErgClawControl: ERG 系列夹爪
    # - EpgClawControl: EPG 系列夹爪
    # - RgClawControl: RG 系列夹爪
    # - 等等...
    
    # 示例：使用 ERG 系列夹爪
    try:
        # 创建夹爪控制对象
        # 参数通常是串口设备，如 '/dev/ttyUSB0' 或 'COM3'
        gripper = ErgClawControl('/dev/ttyUSB0')  # 根据实际串口修改
        
        print("\n1. 初始化夹爪...")
        gripper.initialize()
        time.sleep(1)
        
        print("\n2. 打开夹爪...")
        gripper.open()
        time.sleep(2)
        
        print("\n3. 关闭夹爪...")
        gripper.close()
        time.sleep(2)
        
        print("\n4. 设置夹爪位置 (0-100%)...")
        gripper.set_position(50)  # 50% 开度
        time.sleep(2)
        
        print("\n5. 读取夹爪状态...")
        position = gripper.get_position()
        force = gripper.get_force()
        print(f"  位置: {position}%")
        print(f"  力: {force}")
        
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        print("\n请检查:")
        print("  1. 夹爪是否正确连接")
        print("  2. 串口设备路径是否正确")
        print("  3. 是否有串口访问权限 (sudo usermod -aG dialout $USER)")


def test_with_robot_arm():
    """集成到机械臂控制中使用"""
    print("=" * 60)
    print("夹爪 + 机械臂集成示例")
    print("=" * 60)
    
    # 创建夹爪控制对象
    gripper = ErgClawControl('/dev/ttyUSB0')
    gripper.initialize()
    
    # 在机械臂控制中使用
    # 示例：抓取物体
    print("\n执行抓取...")
    gripper.open()
    time.sleep(1)
    # [机械臂移动到物体上方]
    gripper.close()
    time.sleep(1)
    # [机械臂抓取物体]
    
    # 示例：释放物体
    print("\n执行释放...")
    # [机械臂移动到目标位置]
    gripper.open()
    time.sleep(1)


def list_available_grippers():
    """列出所有可用的夹爪类型"""
    print("=" * 60)
    print("可用的夹爪类型:")
    print("=" * 60)
    
    gripper_types = [
        ('ErgClawControl', 'ERG 系列夹爪'),
        ('ErgClawControl_2', 'ERG 系列夹爪 (变体2)'),
        ('ErgClawControl_08', 'ERG 系列夹爪 (变体08)'),
        ('EpgClawControl', 'EPG 系列夹爪'),
        ('EpgLClawControl', 'EPG-L 系列夹爪'),
        ('EpgHpClawControl', 'EPG-HP 系列夹爪'),
        ('LepgClawControl', 'LEPG 系列夹爪'),
        ('IntegrationEpgClawControl', 'EPG 集成夹爪'),
        ('RgClawControl', 'RG 系列夹爪'),
        ('RcgClawControl', 'RCG 系列夹爪'),
        ('ZrgClawControl', 'ZRG 系列夹爪'),
        ('ElsClawControl', 'ELS 系列夹爪'),
        ('EvsClawControl_01', 'EVS 系列夹爪 (变体01)'),
        ('EvsClawControl_08', 'EVS 系列夹爪 (变体08)'),
    ]
    
    for class_name, description in gripper_types:
        print(f"  • {class_name:30s} - {description}")
    
    print("\n使用方法:")
    print("  from jodellSdk.jodellSdkDemo import ErgClawControl")
    print("  gripper = ErgClawControl('/dev/ttyUSB0')")
    print("  gripper.initialize()")
    print("  gripper.open()  # 打开夹爪")
    print("  gripper.close() # 关闭夹爪")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'list':
        list_available_grippers()
    elif len(sys.argv) > 1 and sys.argv[1] == 'test':
        test_gripper_basic()
    else:
        print("用法:")
        print("  python gripper_control_demo.py list  # 列出所有夹爪类型")
        print("  python gripper_control_demo.py test  # 运行测试")
        print("\n提示: 编辑此文件，修改夹爪型号和串口路径")
