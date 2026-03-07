#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

namespace {

struct Config {
  std::string robot_ip = "192.168.0.160";
  std::string local_ip = "";
  bool enable_gripper = true;
  double gripper_threshold = 0.5;
  unsigned int gripper_board = 2;
  unsigned int gripper_di1_port = 0;
  unsigned int gripper_di2_port = 1;
  double follow_scale = 0.35;
  double cmd_timeout = 0.25;
  double filter_freq = 15.0;
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

void warn_ec(const error_code &ec, const std::string &ctx) {
  if (ec) {
    std::cerr << "WARN " << ctx << ":" << ec.message() << std::endl;
  }
}

void set_gripper(xMateErProRobot &robot, const Config &cfg, double value) {
  if (!cfg.enable_gripper) {
    return;
  }

  error_code ec;
  const bool open_like = value >= cfg.gripper_threshold;
  robot.setDO(cfg.gripper_board, cfg.gripper_di1_port, true, ec);
  if (ec) {
    std::cerr << "WARN setDO_di1:" << ec.message() << std::endl;
    return;
  }
  robot.setDO(cfg.gripper_board, cfg.gripper_di2_port, !open_like, ec);
  if (ec) {
    std::cerr << "WARN setDO_di2:" << ec.message() << std::endl;
  }
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
    else if (arg == "--disable-gripper") cfg.enable_gripper = false;
    else if (arg == "--gripper-threshold") cfg.gripper_threshold = std::stod(need_val(arg));
    else if (arg == "--gripper-board") cfg.gripper_board = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di1-port") cfg.gripper_di1_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di2-port") cfg.gripper_di2_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--follow-scale") cfg.follow_scale = std::stod(need_val(arg));
    else if (arg == "--cmd-timeout") cfg.cmd_timeout = std::stod(need_val(arg));
    else if (arg == "--filter-freq") cfg.filter_freq = std::stod(need_val(arg));
    else if (arg == "--max-pos-speed" || arg == "--max-rot-speed" ||
             arg == "--max-pos-accel" || arg == "--max-rot-accel" ||
             arg == "--speed" || arg == "--zone") {
      (void)need_val(arg);
    }
    else return false;
  }
  return true;
}

Eigen::Transform<double, 3, Eigen::Isometry> posture_to_transform(const std::array<double, 6> &posture) {
  std::array<double, 16> tf_array{};
  Utils::postureToTransArray(posture, tf_array);
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      mat(r, c) = tf_array[4 * r + c];
    }
  }
  Eigen::Transform<double, 3, Eigen::Isometry> tf;
  tf.matrix() = mat;
  return tf;
}

bool read_current_posture(xMateErProRobot &robot, std::array<double, 6> &posture) {
  try {
    std::array<double, 16> measured{};
    robot.getStateData(RtSupportedFields::tcpPose_m, measured);
    Utils::transArrayToPosture(measured, posture);
    return true;
  } catch (...) {
    return false;
  }
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
  robot.setRtNetworkTolerance(40.0, ec);
  warn_ec(ec, "setRtNetworkTolerance");
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  if (!check_ec(ec, "setMotionControlMode")) return 1;
  robot.setPowerState(true, ec);
  if (!check_ec(ec, "setPowerState")) return 1;
  robot.clearServoAlarm(ec);
  warn_ec(ec, "clearServoAlarm");

  try {
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m});
    robot.updateRobotState(std::chrono::milliseconds(50));
  } catch (const std::exception &e) {
    print_err(std::string("startReceiveRobotState:") + e.what());
    return 1;
  }

  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
    if (!rtCon) {
      print_err("getRtMotionController:null");
      return 1;
    }
  } catch (const std::exception &e) {
    print_err(std::string("getRtMotionController:") + e.what());
    return 1;
  }

  rtCon->setFilterLimit(false, cfg.filter_freq);

  auto model = robot.model();
  FollowPosition<7> follow_pose(robot, model);
  follow_pose.setScale(cfg.follow_scale);

  std::atomic<bool> follow_started(false);
  std::array<double, 6> current_posture{};
  std::mutex state_mutex;
  std::chrono::steady_clock::time_point last_exec_tp = std::chrono::steady_clock::now();

  if (read_current_posture(robot, current_posture)) {
    try {
      follow_pose.start(posture_to_transform(current_posture));
      follow_started.store(true);
    } catch (const std::exception &e) {
      print_err(std::string("follow_start:") + e.what());
    }
  }

  std::cout << "READY" << std::endl;

  std::string line;
  double gripper_pos = 1.0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;

    if (read_current_posture(robot, current_posture)) {
      std::lock_guard<std::mutex> lock(state_mutex);
    }

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "RESET") {
      if (read_current_posture(robot, current_posture)) {
        try {
          if (follow_started.load()) {
            follow_pose.stop();
            follow_started.store(false);
          }
          follow_pose.setScale(cfg.follow_scale);
          follow_pose.start(posture_to_transform(current_posture));
          follow_started.store(true);
        } catch (const std::exception &e) {
          print_err(std::string("follow_reset:") + e.what());
        }
      }
      last_exec_tp = std::chrono::steady_clock::now();
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

      std::array<double, 6> target_posture = {x, y, z, rx, ry, rz};
      try {
        if (!follow_started.load()) {
          follow_pose.setScale(cfg.follow_scale);
          follow_pose.start(posture_to_transform(target_posture));
          follow_started.store(true);
        } else {
          follow_pose.update(posture_to_transform(target_posture));
        }
      } catch (const std::exception &e) {
        print_err(std::string("follow_update:") + e.what());
        continue;
      }

      last_exec_tp = std::chrono::steady_clock::now();
      gripper_pos = grip;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "STATE") {
      std::array<double, 6> posture{};
      if (!read_current_posture(robot, posture)) {
        posture = current_posture;
      }
      std::cout << "STATE "
                << posture[0] << " " << posture[1] << " " << posture[2] << " "
                << posture[3] << " " << posture[4] << " " << posture[5] << " "
                << gripper_pos << std::endl;
      continue;
    }

    if (cmd == "CLOSE") {
      if (follow_started.load()) {
        try {
          follow_pose.stop();
        } catch (...) {
        }
      }
      std::cout << "OK" << std::endl;
      break;
    }

    print_err("unknown_cmd");
  }

  robot.stopReceiveRobotState();
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  warn_ec(ec, "setMotionControlMode(NrtCommand)");

  return 0;
}
