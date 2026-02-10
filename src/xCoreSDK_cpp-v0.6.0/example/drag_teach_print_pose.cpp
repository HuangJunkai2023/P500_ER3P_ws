/**
 * @file drag_teach_print_pose.cpp
 * @brief 拖动示教并实时打印末端位姿 xyz+rpy
 */

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;

int main() {
  const std::string robot_ip = "192.168.0.160";
  xMateErProRobot robot;

  try {
    // 1) 连接
    robot.connectToRobot(robot_ip);
  } catch (const std::exception &e) {
    std::cerr << "连接失败: " << e.what() << std::endl;
    return -1;
  }

  error_code ec;

  // 2) 清除错误（伺服报警 + 急停恢复）
  robot.clearServoAlarm(ec);
  if (ec) {
    print(std::cerr, "clearServoAlarm 失败:", ec);
  }
  ec.clear();
  robot.recoverState(1, ec);
  if (ec) {
    print(std::cerr, "recoverState(1) 失败:", ec);
  }

  // 3) 使能（上电）
  robot.setOperateMode(OperateMode::manual, ec);
  robot.setPowerState(true, ec);
  if (ec) {
    print(std::cerr, "上电失败:", ec);
    return -1;
  }
  print(std::cout, "已上电，准备切换拖动示教");

  // SDK要求开启拖动前处于手动下电状态
  robot.setPowerState(false, ec);
  if (ec) {
    print(std::cerr, "切换到拖动前下电失败:", ec);
    return -1;
  }

  // 4) 开启示教（拖动）
  robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec, true);
  if (ec) {
    print(std::cerr, "enableDrag 失败:", ec);
    return -1;
  }

  std::atomic_bool running{true};
  std::thread wait_exit([&running]() {
    std::cout << "拖动中，按回车退出..." << std::endl;
    std::cin.get();
    running = false;
  });

  // 5) 持续打印末端位姿（单位: m, rad）
  while (running) {
    auto pose = robot.cartPosture(CoordinateType::endInRef, ec);
    if (!ec) {
      std::cout << "xyzrpy: "
                << pose.trans[0] << ", "
                << pose.trans[1] << ", "
                << pose.trans[2] << ", "
                << pose.rpy[0] << ", "
                << pose.rpy[1] << ", "
                << pose.rpy[2] << std::endl;
    } else {
      print(std::cerr, "读取位姿失败:", ec);
      ec.clear();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (wait_exit.joinable()) {
    wait_exit.join();
  }

  // 6) 退出拖动
  robot.disableDrag(ec);
  if (ec) {
    print(std::cerr, "disableDrag 失败:", ec);
  }

  // 7) 失能（下电）
  ec.clear();
  robot.setPowerState(false, ec);
  if (ec) {
    print(std::cerr, "下电失败:", ec);
    return -1;
  }

  print(std::cout, "流程结束");
  return 0;
}
