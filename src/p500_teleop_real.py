#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P500 + 珞石机械臂手机示教主程序
完全参考 tidybot2/main.py，只修改环境导入
"""

import sys
import os
import argparse
import time
from itertools import count

# 添加 tidybot2 路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, 'tidybot2'))

# 导入 tidybot2 的示教策略和常量
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeWriter
from policies import TeleopPolicy, RemotePolicy

# 导入我们的真实环境
from p500_real_env import P500RealEnv


def should_save_episode(writer):
    """询问是否保存回合数据"""
    if len(writer) == 0:
        print('丢弃空回合')
        return False

    # 询问用户
    while True:
        user_input = input('保存回合数据? (y/n): ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('丢弃回合数据')
            return False
        print('无效输入')


def run_episode(env, policy, writer=None):
    """
    运行一个示教回合
    完全参考 tidybot2/main.py 的 run_episode 函数
    """
    # 重置环境
    print('正在重置环境...')
    env.reset()
    print('环境已重置')

    # 等待用户在 web app 点击 "Start episode"
    print('\n' + '=' * 70)
    print('在手机浏览器中点击 "Start episode" 开始新回合')
    print('=' * 70)
    policy.reset()
    print('\n' + '=' * 70)
    print('新回合已开始')
    print('=' * 70)
    print('操作提示:')
    print('  • 触摸屏幕左侧 (90% 区域) → 控制机械臂末端')
    print('  • 上下滑动 → 控制夹爪开合')
    print('  • 触摸屏幕右侧 (10% 区域) → 控制底盘移动')
    print('=' * 70 + '\n')

    episode_ended = False
    start_time = time.time()
    for step_idx in count():
        # 严格控制频率
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # 获取最新观测
        obs = env.get_obs()

        # 获取动作
        action = policy.step(obs)

        # 如果示教未启用，跳过
        if action is None:
            continue

        # 执行有效动作
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # 记录执行的动作
                writer.step(obs, action)

        # 回合结束
        elif not episode_ended and action == 'end_episode':
            episode_ended = True
            duration = time.time() - start_time
            print(f'\n回合已结束 (时长: {duration:.1f}秒)')

            if writer is not None and should_save_episode(writer):
                # 在后台线程保存到磁盘
                writer.flush_async()

            print('\n示教模式已激活。在 web app 中点击 "Reset env" 继续。\n')

        # 准备环境重置
        elif action == 'reset_env':
            break

    if writer is not None:
        # 等待 writer 保存完成
        writer.wait_for_flush()


def main(args):
    """主函数"""
    print('\n' + '=' * 70)
    print('P500 + 珞石机械臂手机示教系统')
    print('基于 tidybot2 框架')
    print('=' * 70)
    
    # 创建环境
    print('\n[步骤 1/3] 连接机器人...')
    print('提示: 请确保已启动 RPC 服务器:')
    print('  终端 1: python p500_base_server.py')
    print('  终端 2: python rokae_arm_server.py\n')
    
    try:
        env = P500RealEnv(
            chassis_ip=args.chassis_ip,
            arm_ip=args.arm_ip
        )
    except Exception as e:
        print(f'\n错误: 无法连接到机器人环境')
        print(f'详细: {e}\n')
        return

    # 创建策略
    print('[步骤 2/3] 创建示教策略...')
    if args.teleop:
        policy = TeleopPolicy(use_ssl=args.ssl)
    else:
        policy = RemotePolicy()
    print('  ✓ 策略已创建')

    print('\n[步骤 3/3] 启动示教服务器...')
    print('\n' + '=' * 70)
    print('✓ 系统已就绪！')
    print('=' * 70)
    print('\n下一步:')
    print('  1. 用手机连接到与机器人相同的 WiFi')
    print('  2. 在手机浏览器中打开上方显示的地址')
    print('  3. 允许浏览器访问摄像头和传感器')
    print('  4. 点击 "Start episode" 开始示教')
    print('\n提示: 按 Ctrl+C 退出程序\n')

    try:
        # 主循环
        while True:
            writer = EpisodeWriter(args.output_dir) if args.save else None
            run_episode(env, policy, writer)
    
    except KeyboardInterrupt:
        print('\n\n收到停止信号，正在退出...')
    
    finally:
        env.close()
        print('程序已退出\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='P500 + 珞石机械臂手机示教系统',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
使用示例:
  # 基本使用（手机示教）
  python p500_teleop_real.py --teleop
  
  # 指定设备 IP
  python p500_teleop_real.py --teleop --arm-ip 192.168.1.160 --chassis-ip 192.168.1.100
  
  # 保存示教数据
  python p500_teleop_real.py --teleop --save --output-dir data/demos
  
  # 使用 HTTPS (某些设备需要)
  python p500_teleop_real.py --teleop --ssl
  
  # 运行训练好的策略
  python p500_teleop_real.py

注意:
  运行前必须先启动 RPC 服务器:
    终端 1: python p500_base_server.py
    终端 2: python rokae_arm_server.py
        '''
    )
    
    # 设备配置
    parser.add_argument('--arm-ip', default='192.168.1.160',
                        help='机械臂控制器 IP (默认: 192.168.1.160)')
    parser.add_argument('--chassis-ip', default='192.168.1.100',
                        help='底盘控制器 IP (默认: 192.168.1.100)')
    
    # 模式选择
    parser.add_argument('--teleop', action='store_true',
                        help='手机示教模式')
    
    # 功能选项
    parser.add_argument('--save', action='store_true',
                        help='保存示教数据')
    parser.add_argument('--output-dir', default='data/demos',
                        help='数据保存目录 (默认: data/demos)')
    parser.add_argument('--ssl', action='store_true',
                        help='使用 HTTPS (某些设备的 WebXR 需要)')
    
    args = parser.parse_args()
    main(args)
