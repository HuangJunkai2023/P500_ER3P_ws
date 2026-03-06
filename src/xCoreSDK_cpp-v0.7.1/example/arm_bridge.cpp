#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

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
  double max_pos_speed = 0.18;
  double max_rot_speed = 0.90;
  double max_pos_accel = 0.90;
  double max_rot_accel = 4.50;
  double cmd_timeout = 0.25;
};

constexpr double kDt = 0.001;

double wrap_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

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
    return;
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
    else if (arg == "--max-pos-speed") cfg.max_pos_speed = std::stod(need_val(arg));
    else if (arg == "--max-rot-speed") cfg.max_rot_speed = std::stod(need_val(arg));
    else if (arg == "--max-pos-accel") cfg.max_pos_accel = std::stod(need_val(arg));
    else if (arg == "--max-rot-accel") cfg.max_rot_accel = std::stod(need_val(arg));
    else if (arg == "--cmd-timeout") cfg.cmd_timeout = std::stod(need_val(arg));
    else if (arg == "--speed" || arg == "--zone") { (void)need_val(arg); }
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
  robot.setRtNetworkTolerance(40.0, ec);
  warn_ec(ec, "setRtNetworkTolerance");
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  if (!check_ec(ec, "setMotionControlMode")) return 1;
  robot.setPowerState(true, ec);
  if (!check_ec(ec, "setPowerState")) return 1;
  robot.clearServoAlarm(ec);
  warn_ec(ec, "clearServoAlarm");

  std::array<double, 16> target_pose_m{};
  std::array<double, 6> target_posture{};
  std::array<double, 6> traj_pos{};
  std::array<double, 6> traj_vel{};
  std::array<double, 6> current_posture{};
  std::mutex pose_mutex;
  std::atomic<bool> finish_requested(false);
  std::atomic<bool> loop_alive(false);
  std::chrono::steady_clock::time_point last_exec_tp = std::chrono::steady_clock::now();
  std::string loop_error;
  std::mutex loop_error_mutex;

  try {
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m});
    robot.updateRobotState(std::chrono::milliseconds(50));
    robot.getStateData(RtSupportedFields::tcpPose_m, target_pose_m);
    Utils::transArrayToPosture(target_pose_m, current_posture);
    target_posture = current_posture;
    traj_pos = current_posture;
    traj_vel.fill(0.0);
    last_exec_tp = std::chrono::steady_clock::now();
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

  std::function<CartesianPosition()> callback = [&]() {
    std::array<double, 6> local_target_posture{};
    std::array<double, 6> local_traj_pos{};
    std::array<double, 6> local_traj_vel{};
    {
      std::lock_guard<std::mutex> lock(pose_mutex);
      local_target_posture = target_posture;
      local_traj_pos = traj_pos;
      local_traj_vel = traj_vel;
    }

    std::array<double, 16> measured{};
    try {
      robot.getStateData(RtSupportedFields::tcpPose_m, measured);
      std::array<double, 6> measured_posture{};
      Utils::transArrayToPosture(measured, measured_posture);
      std::lock_guard<std::mutex> lock(pose_mutex);
      current_posture = measured_posture;
    } catch (...) {
    }

    const double max_pos_step = cfg.max_pos_speed * kDt;
    const double max_rot_step = cfg.max_rot_speed * kDt;
    const double max_pos_acc_step = cfg.max_pos_accel * kDt;
    const double max_rot_acc_step = cfg.max_rot_accel * kDt;

    const auto now_tp = std::chrono::steady_clock::now();
    const double since_exec = std::chrono::duration<double>(now_tp - last_exec_tp).count();
    if (since_exec > cfg.cmd_timeout) {
      local_target_posture = local_traj_pos;
    }

    for (size_t i = 0; i < 3; ++i) {
      double err = local_target_posture[i] - local_traj_pos[i];
      double dir = (err >= 0.0) ? 1.0 : -1.0;
      double v_stop = std::sqrt(2.0 * cfg.max_pos_accel * std::abs(err));
      double vel_des = dir * std::min(cfg.max_pos_speed, v_stop);
      double vel_delta = vel_des - local_traj_vel[i];
      vel_delta = clamp(vel_delta, -max_pos_acc_step, max_pos_acc_step);
      local_traj_vel[i] += vel_delta;
      local_traj_vel[i] = clamp(local_traj_vel[i], -cfg.max_pos_speed, cfg.max_pos_speed);
      double delta = local_traj_vel[i] * kDt;
      delta = clamp(delta, -max_pos_step, max_pos_step);
      if (std::abs(delta) > std::abs(err)) {
        delta = err;
        local_traj_vel[i] = 0.0;
      }
      local_traj_pos[i] += delta;
    }

    for (size_t i = 3; i < 6; ++i) {
      double err = wrap_angle(local_target_posture[i] - local_traj_pos[i]);
      double dir = (err >= 0.0) ? 1.0 : -1.0;
      double v_stop = std::sqrt(2.0 * cfg.max_rot_accel * std::abs(err));
      double vel_des = dir * std::min(cfg.max_rot_speed, v_stop);
      double vel_delta = vel_des - local_traj_vel[i];
      vel_delta = clamp(vel_delta, -max_rot_acc_step, max_rot_acc_step);
      local_traj_vel[i] += vel_delta;
      local_traj_vel[i] = clamp(local_traj_vel[i], -cfg.max_rot_speed, cfg.max_rot_speed);
      double delta = local_traj_vel[i] * kDt;
      delta = clamp(delta, -max_rot_step, max_rot_step);
      if (std::abs(delta) > std::abs(err)) {
        delta = err;
        local_traj_vel[i] = 0.0;
      }
      local_traj_pos[i] = wrap_angle(local_traj_pos[i] + delta);
    }

    std::array<double, 16> local_target{};
    Utils::postureToTransArray(local_traj_pos, local_target);
    {
      std::lock_guard<std::mutex> lock(pose_mutex);
      traj_pos = local_traj_pos;
      traj_vel = local_traj_vel;
    }

    CartesianPosition cmd;
    cmd.pos = local_target;
    if (finish_requested.load()) {
      cmd.setFinished();
    }
    return cmd;
  };

  try {
    rtCon->setControlLoop(callback, 0, true);
    rtCon->startMove(RtControllerMode::cartesianPosition);
  } catch (const std::exception &e) {
    print_err(std::string("startMove:") + e.what());
    return 1;
  }

  std::thread loop_thread([&]() {
    loop_alive.store(true);
    try {
      rtCon->startLoop(true);
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(loop_error_mutex);
      loop_error = e.what();
    }
    loop_alive.store(false);
  });

  std::cout << "READY" << std::endl;

  std::string line;
  double gripper_pos = 1.0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;

    {
      std::lock_guard<std::mutex> lock(loop_error_mutex);
      if (!loop_error.empty()) {
        print_err(std::string("rt_loop:") + loop_error);
        loop_error.clear();
      }
    }

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "RESET") {
      std::array<double, 16> measured{};
      try {
        robot.getStateData(RtSupportedFields::tcpPose_m, measured);
        std::lock_guard<std::mutex> lock(pose_mutex);
        target_pose_m = measured;
        Utils::transArrayToPosture(measured, current_posture);
        target_posture = current_posture;
        traj_pos = current_posture;
        traj_vel.fill(0.0);
      } catch (...) {
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

      std::array<double, 6> posture = {x, y, z, rx, ry, rz};
      std::array<double, 16> target{};
      Utils::postureToTransArray(posture, target);
      {
        std::lock_guard<std::mutex> lock(pose_mutex);
        target_pose_m = target;
        target_posture = posture;
      }
      last_exec_tp = std::chrono::steady_clock::now();

      gripper_pos = grip;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "STATE") {
      std::array<double, 6> posture{};
      {
        std::lock_guard<std::mutex> lock(pose_mutex);
        posture = current_posture;
      }
      std::cout << "STATE "
                << posture[0] << " " << posture[1] << " " << posture[2] << " "
                << posture[3] << " " << posture[4] << " " << posture[5] << " "
                << gripper_pos << std::endl;
      continue;
    }

    if (cmd == "CLOSE") {
      finish_requested.store(true);
      std::cout << "OK" << std::endl;
      break;
    }

    print_err("unknown_cmd");
  }

  if (loop_thread.joinable()) {
    loop_thread.join();
  }

  robot.stopReceiveRobotState();
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  warn_ec(ec, "setMotionControlMode(NrtCommand)");

  return 0;
}
