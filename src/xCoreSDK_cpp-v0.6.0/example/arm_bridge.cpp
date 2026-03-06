#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rokae/robot.h"

using namespace rokae;

namespace {

struct Config {
  std::string robot_ip = "192.168.0.160";
  std::string local_ip = "";
  double speed = 300.0;
  double zone = 10.0;
  double gripper_threshold = 0.5;
  unsigned int gripper_board = 2;
  unsigned int gripper_di1_port = 0;
  unsigned int gripper_di2_port = 1;
};

void print_err(const std::string &msg) {
  std::cout << "ERR " << msg << std::endl;
}

bool check_ec(const error_code &ec, const std::string &ctx) {
  if (ec) {
    print_err(ctx + ":" + ec.message());
    return false;
  }
  return true;
}

void set_gripper(xMateErProRobot &robot, const Config &cfg, double value) {
  error_code ec;
  const bool open_like = value >= cfg.gripper_threshold;
  robot.setDO(cfg.gripper_board, cfg.gripper_di1_port, true, ec);
  if (!check_ec(ec, "setDO_di1")) return;
  robot.setDO(cfg.gripper_board, cfg.gripper_di2_port, !open_like, ec);
  if (!check_ec(ec, "setDO_di2")) return;
}

bool parse_args(int argc, char **argv, Config &cfg) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto need_val = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
      return std::string(argv[++i]);
    };

    if (arg == "--robot-ip") cfg.robot_ip = need_val(arg);
    else if (arg == "--local-ip") cfg.local_ip = need_val(arg);
    else if (arg == "--speed") cfg.speed = std::stod(need_val(arg));
    else if (arg == "--zone") cfg.zone = std::stod(need_val(arg));
    else if (arg == "--gripper-threshold") cfg.gripper_threshold = std::stod(need_val(arg));
    else if (arg == "--gripper-board") cfg.gripper_board = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di1-port") cfg.gripper_di1_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di2-port") cfg.gripper_di2_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else return false;
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  Config cfg;
  try {
    if (!parse_args(argc, argv, cfg)) {
      std::cout << "ERR bad_args" << std::endl;
      return 2;
    }
  } catch (const std::exception &e) {
    print_err(std::string("arg_parse:") + e.what());
    return 2;
  }

  xMateErProRobot robot;
  try {
    if (cfg.local_ip.empty()) robot.connectToRobot(cfg.robot_ip);
    else robot.connectToRobot(cfg.robot_ip, cfg.local_ip);
  } catch (const std::exception &e) {
    print_err(std::string("connect:") + e.what());
    return 1;
  }

  error_code ec;
  robot.setOperateMode(OperateMode::automatic, ec);
  if (!check_ec(ec, "setOperateMode")) return 1;
  robot.setPowerState(true, ec);
  if (!check_ec(ec, "setPowerState")) return 1;
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if (!check_ec(ec, "setMotionControlMode")) return 1;
  robot.clearServoAlarm(ec);
  if (!check_ec(ec, "clearServoAlarm")) return 1;

  std::cout << "READY" << std::endl;

  std::string line;
  double gripper_pos = 1.0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "RESET") {
      robot.moveReset(ec);
      if (!check_ec(ec, "moveReset")) continue;
      robot.clearServoAlarm(ec);
      if (!check_ec(ec, "clearServoAlarm")) continue;
      gripper_pos = 1.0;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "EXEC") {
      double x, y, z, rx, ry, rz, grip;
      if (!(iss >> x >> y >> z >> rx >> ry >> rz >> grip)) {
        print_err("bad_exec_args");
        continue;
      }

      CartesianPosition target({x, y, z, rx, ry, rz});
      MoveJCommand move_cmd(target, cfg.speed, cfg.zone);

      std::string cmd_id;
      robot.moveReset(ec);
      if (!check_ec(ec, "moveReset")) continue;
      robot.moveAppend({move_cmd}, cmd_id, ec);
      if (!check_ec(ec, "moveAppend")) continue;
      robot.moveStart(ec);
      if (!check_ec(ec, "moveStart")) continue;

      gripper_pos = grip;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "STATE") {
      auto posture = robot.posture(CoordinateType::endInRef, ec);
      if (!check_ec(ec, "posture")) continue;
      std::cout << "STATE "
                << posture[0] << " " << posture[1] << " " << posture[2] << " "
                << posture[3] << " " << posture[4] << " " << posture[5] << " "
                << gripper_pos << std::endl;
      continue;
    }

    if (cmd == "CLOSE") {
      robot.moveReset(ec);
      std::cout << "OK" << std::endl;
      break;
    }

    print_err("unknown_cmd");
  }

  return 0;
}
