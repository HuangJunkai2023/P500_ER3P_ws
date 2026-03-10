#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
  double tcp_offset_z = 0.10;
  double move_speed = 300.0;
  double move_zone = 10.0;
  std::array<double, 7> preset_joints_deg = {0.0, 30.0, 0.0, 60.0, 0.0, 90.0, 0.0};
};

bool parse_csv7(const std::string &text, std::array<double, 7> &out) {
  std::stringstream ss(text);
  std::string item;
  std::vector<double> vals;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    vals.push_back(std::stod(item));
  }
  if (vals.size() != 7) return false;
  for (size_t i = 0; i < 7; ++i) out[i] = vals[i];
  return true;
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
    else if (arg == "--tcp-offset-z") cfg.tcp_offset_z = std::stod(need_val(arg));
    else if (arg == "--speed") cfg.move_speed = std::stod(need_val(arg));
    else if (arg == "--zone") cfg.move_zone = std::stod(need_val(arg));
    else if (arg == "--preset-joints-deg") {
      if (!parse_csv7(need_val(arg), cfg.preset_joints_deg)) {
        throw std::runtime_error("bad value for --preset-joints-deg, expected 7 comma-separated numbers");
      }
    }
    else if (arg == "--max-pos-speed" || arg == "--max-rot-speed" ||
             arg == "--max-pos-accel" || arg == "--max-rot-accel") {
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

Eigen::Transform<double, 3, Eigen::Isometry> trans_array_to_transform(const std::array<double, 16> &tf_array) {
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

Eigen::Transform<double, 3, Eigen::Isometry> pose_quat_to_transform(double x, double y, double z,
                                                                     double qx, double qy, double qz, double qw) {
  Eigen::Quaterniond quat(qw, qx, qy, qz);
  if (quat.norm() < 1e-12) {
    quat = Eigen::Quaterniond::Identity();
  } else {
    quat.normalize();
  }

  Eigen::Transform<double, 3, Eigen::Isometry> tf = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  tf.linear() = quat.toRotationMatrix();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Transform<double, 3, Eigen::Isometry> flange_to_tool_tf(double offset_z) {
  Eigen::Transform<double, 3, Eigen::Isometry> tf = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, offset_z);
  return tf;
}

std::array<double, 6> transform_to_posture(const Eigen::Transform<double, 3, Eigen::Isometry> &tf) {
  std::array<double, 16> tf_array{};
  const auto &mat = tf.matrix();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      tf_array[4 * r + c] = mat(r, c);
    }
  }
  std::array<double, 6> posture{};
  Utils::transArrayToPosture(tf_array, posture);
  return posture;
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

bool wait_robot_idle(xMateErProRobot &robot, std::chrono::milliseconds timeout) {
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    error_code ec;
    auto st = robot.operationState(ec);
    if (!ec && (st == OperationState::idle || st == OperationState::unknown)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
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

  const auto tf_f2t = flange_to_tool_tf(cfg.tcp_offset_z);
  const auto tf_t2f = tf_f2t.inverse();

  auto model = robot.model();
  FollowPosition<7> follow_pose(robot, model);
  follow_pose.setScale(cfg.follow_scale);

  const auto preset_joint_rad = Utils::degToRad(cfg.preset_joints_deg);
  const JointPosition preset_joint_pos(std::vector<double>(preset_joint_rad.begin(), preset_joint_rad.end()));
  const auto preset_tf_arr = model.getCartPose(preset_joint_rad);
  const auto preset_target_tf = trans_array_to_transform(preset_tf_arr);

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

  auto restart_follow_from_current = [&]() -> bool {
    if (!read_current_posture(robot, current_posture)) {
      print_err("read_current_posture");
      return false;
    }
    try {
      follow_pose.setScale(cfg.follow_scale);
      follow_pose.start(posture_to_transform(current_posture));
      follow_started.store(true);
      return true;
    } catch (const std::exception &e) {
      print_err(std::string("follow_restart:") + e.what());
      return false;
    }
  };

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
      try {
        if (follow_started.load()) {
          follow_pose.stop();
          follow_started.store(false);
        }
      } catch (const std::exception &e) {
        print_err(std::string("follow_stop:") + e.what());
      }

      // Use MoveAbsJ for reset so the arm moves smoothly in joint space to teleop preset.
      robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
      if (!check_ec(ec, "setMotionControlMode(NrtCommand)")) continue;
      robot.setOperateMode(OperateMode::automatic, ec);
      if (!check_ec(ec, "setOperateMode")) continue;
      robot.setPowerState(true, ec);
      if (!check_ec(ec, "setPowerState")) continue;

      MoveAbsJCommand move_absj(preset_joint_pos, cfg.move_speed, cfg.move_zone);
      robot.executeCommand({move_absj}, ec);
      if (!check_ec(ec, "executeCommand(MoveAbsJCommand)")) continue;
      if (!wait_robot_idle(robot, std::chrono::seconds(30))) {
        print_err("wait_robot_idle_timeout");
        continue;
      }

      robot.setRtNetworkTolerance(40.0, ec);
      warn_ec(ec, "setRtNetworkTolerance");
      robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
      if (!check_ec(ec, "setMotionControlMode(RtCommand)")) continue;
      robot.setOperateMode(OperateMode::automatic, ec);
      if (!check_ec(ec, "setOperateMode")) continue;
      robot.setPowerState(true, ec);
      if (!check_ec(ec, "setPowerState")) continue;

      if (!restart_follow_from_current()) {
        continue;
      }

      last_exec_tp = std::chrono::steady_clock::now();
      gripper_pos = 1.0;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "PRESET") {
      try {
        if (!follow_started.load()) {
          follow_pose.setScale(cfg.follow_scale);
          follow_pose.start(preset_target_tf);
          follow_started.store(true);
        } else {
          follow_pose.update(preset_target_tf);
        }
      } catch (const std::exception &e) {
        print_err(std::string("preset_update:") + e.what());
        continue;
      }
      last_exec_tp = std::chrono::steady_clock::now();
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "EXEC") {
      double x, y, z;
      if (!(iss >> x >> y >> z)) {
        print_err("bad_exec_args");
        continue;
      }

      std::vector<double> rest;
      double v;
      while (iss >> v) {
        rest.push_back(v);
      }

      Eigen::Transform<double, 3, Eigen::Isometry> target_tf;
      double grip = 0.0;
      if (rest.size() == 5) {
        // New format: x y z qx qy qz qw grip
        target_tf = pose_quat_to_transform(x, y, z, rest[0], rest[1], rest[2], rest[3]);
        grip = rest[4];
      } else if (rest.size() == 4) {
        // Legacy format: x y z rx ry rz grip
        std::array<double, 6> target_posture = {x, y, z, rest[0], rest[1], rest[2]};
        target_tf = posture_to_transform(target_posture);
        grip = rest[3];
      } else {
        print_err("bad_exec_args");
        continue;
      }

      // External API target is tool center point (TCP at gripper center),
      // while controller follows flange pose.
      target_tf = target_tf * tf_t2f;

      try {
        if (!follow_started.load()) {
          follow_pose.setScale(cfg.follow_scale);
          follow_pose.start(target_tf);
          follow_started.store(true);
        } else {
          follow_pose.update(target_tf);
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

      // Convert reported pose from flange to tool center point.
      const auto flange_tf = posture_to_transform(posture);
      const auto tool_tf = flange_tf * tf_f2t;
      const auto tool_posture = transform_to_posture(tool_tf);

      std::cout << "STATE "
                << tool_posture[0] << " " << tool_posture[1] << " " << tool_posture[2] << " "
                << tool_posture[3] << " " << tool_posture[4] << " " << tool_posture[5] << " "
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
