/**
 * @file joint_position_control.cpp
 * @brief 实时模式 - 轴角度控制。程序适用机型：xMateER3
 *
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "../print_helper.hpp"

using namespace rokae;

/**
 * @brief main program
 */
int main() {
  using namespace std;

  // 创建机器人对象
  rokae::xMateRobot robot;
  std::error_code ec;

  // 连接到机器人
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100"); // 本机地址192.168.0.100
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
    return 0;
  }

  //设置机器人状态
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setPowerState(true, ec);

  try {
    auto rtCon = robot.getRtMotionController().lock();

    // 可选：设置滤波截止频率。若运动中出现异响、轻微抖动等问题，在排除网络波动、轨迹规划不合适等原因后，
    // 可设置滤波来平滑指令，缓解异响。数值越低平滑效果越好
    rtCon->setFilterLimit(true, 10);
    rtCon->setFilterFrequency(10, 10, 10, ec);

    // 可选: 设置要接收数据（每毫秒接收一次机器人关节位置状态）
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});

    // 定义和初始化运动控制时间和角度
    double time = 0;

    std::array<double, 6> jntPos{};
    std::array<double, 6> q_drag_xm3 = {0, M_PI/6, M_PI/3, 0, M_PI/2, 0};

    // 定义回调函数
    std::function<JointPosition()> callback = [&, rtCon](){
      time += 0.001;
      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time));
      JointPosition cmd = {{jntPos[0] + delta_angle, jntPos[1] + delta_angle,
                            jntPos[2] - delta_angle,
                            jntPos[3] + delta_angle, jntPos[4] - delta_angle,
                            jntPos[5] + delta_angle}};

      if(time > 60) {
        cmd.setFinished(); // 60秒后结束
      }
      return cmd;
    };

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.3, robot.jointPos(ec), q_drag_xm3);

    // 设置回调函数
    rtCon->setControlLoop(callback);
    // 更新起始角度为当前角度
    jntPos = robot.jointPos(ec);
    // 开始轴空间位置控制
    rtCon->startMove(RtControllerMode::jointPosition);
    // 阻塞loop，开始运动
    rtCon->startLoop(true);
    print(std::cout, "控制结束");

    //将控制模式设为空闲并下电
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false,ec);

  } catch (const std::exception &e) {
    // 捕获异常并打印错误信息
    print(std::cerr, e.what());
    robot.setMotionControlMode(MotionControlMode::Idle, ec);
    robot.setPowerState(false,ec);
  }

  return 0;
}
