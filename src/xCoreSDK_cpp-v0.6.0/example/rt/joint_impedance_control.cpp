/**
 * @file joint_impedance_control.cpp
 * @brief 实时模式 - 轴空间阻抗控制。程序适用机型xMateER7 Pro
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
  try {
    std::string ip = "192.168.0.160"; // 机器人地址
    std::error_code ec;
    rokae::xMateErProRobot robot(ip, "192.168.0.100"); // 本机地址192.168.0.100

    robot.setOperateMode(rokae::OperateMode::automatic,ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    auto rtCon = robot.getRtMotionController().lock();

    // 可选：设置要接收数据
    robot.startReceiveRobotState(std::chrono::milliseconds(1), { RtSupportedFields::jointPos_m});

    double time = 0;

    std::array<double, 7> jntPos{};
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};

    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, jntPos, q_drag_xm7p);

    // 回调函数
    std::function<JointPosition(void)> callback = [&jntPos, rtCon, &time] {
      time += 0.001;
      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

      JointPosition cmd(7);
      for(unsigned i = 0; i < cmd.joints.size(); ++i) {
        cmd.joints[i] = jntPos[i] + delta_angle;
      }

      if(time > 60) {
        cmd.setFinished(); // 60秒后结束
      }
      return cmd;
    };

    // 设置轴空间阻抗系数，
    rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
    // 设置回调函数
    rtCon->setControlLoop(callback);
    // 更新起始位置为当前位置
    jntPos = robot.jointPos(ec);
    // 开始轴空间阻抗运动
    rtCon->startMove(RtControllerMode::jointImpedance);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");
  } catch (const std::exception &e) {
    std::cout << e.what();
  }
  return 0;
}
