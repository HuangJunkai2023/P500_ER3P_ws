#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
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

enum class GripperBackend {
  Di,
  Rs485Epg,
};

struct Config {
  std::string robot_ip = "192.168.0.160";
  std::string local_ip = "";
  bool enable_gripper = true;
  GripperBackend gripper_backend = GripperBackend::Di;
  double gripper_threshold = 0.5;
  unsigned int gripper_board = 2;
  unsigned int gripper_di1_port = 0;
  unsigned int gripper_di2_port = 1;
  int gripper_rs485_slave_id = 9;
  bool gripper_rs485_enable_on_start = false;
  int gripper_rs485_init_reg = 0x0100;
  int gripper_rs485_init_value = 0x00A5;
  int gripper_rs485_torque_reg = 0x0101;
  int gripper_rs485_pos_reg = 0x0103;
  int gripper_rs485_speed_reg = 0x0104;
  int gripper_rs485_pos_now_reg = 0x0202;
  int gripper_rs485_force_now_reg = 0x0203;
  int gripper_rs485_open_pos = 0;
  int gripper_rs485_close_pos = 255;
  int gripper_rs485_speed = 255;
  int gripper_rs485_torque = 80;
  double follow_scale = 0.35;
  double cmd_timeout = 0.25;
  double filter_freq = 15.0;
  double tcp_offset_z = 0.10;
  double move_speed = 300.0;
  double move_zone = 10.0;
  double max_joint_speed = 0.35;
  double max_joint_accel = 1.00;
  std::array<double, 7> preset_joints_deg = {0.0, 30.0, 0.0, 60.0, 0.0, 90.0, 0.0};
  std::array<double, 7> joint_impedance = {500.0, 500.0, 500.0, 500.0, 50.0, 50.0, 50.0};
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

int parse_int_auto_base(const std::string &s) {
  return static_cast<int>(std::stol(s, nullptr, 0));
}

int to_signed_int16_word(int value) {
  const int u16 = value & 0xFFFF;
  return (u16 >= 0x8000) ? (u16 - 0x10000) : u16;
}

bool write_modbus_reg(xMateErProRobot &robot, int slave_id, int reg_addr, int value, const std::string &ctx) {
  error_code ec;
  std::vector<int> data = {value};
  robot.XPRWModbusRTUReg(slave_id, 0x06, reg_addr, "int16", 1, data, false, ec);
  if (ec) {
    std::cerr << "WARN " << ctx << ":" << ec.message() << std::endl;
    return false;
  }
  return true;
}

bool write_modbus_regs(xMateErProRobot &robot, int slave_id, int reg_addr, std::vector<int> data, const std::string &ctx) {
  error_code ec;
  robot.XPRWModbusRTUReg(slave_id, 0x10, reg_addr, "int16", static_cast<int>(data.size()), data, false, ec);
  if (ec) {
    std::cerr << "WARN " << ctx << ":" << ec.message() << std::endl;
    return false;
  }
  return true;
}

bool read_modbus_reg(xMateErProRobot &robot, int slave_id, int reg_addr, int &value, const std::string &ctx) {
  error_code ec;
  std::vector<int> data = {0};
  robot.XPRWModbusRTUReg(slave_id, 0x03, reg_addr, "int16", 1, data, false, ec);
  if (ec) {
    std::cerr << "WARN " << ctx << ":" << ec.message() << std::endl;
    return false;
  }
  value = data[0];
  return true;
}

bool read_modbus_input_reg(xMateErProRobot &robot, int slave_id, int reg_addr, int &value, const std::string &ctx) {
  error_code ec;
  std::vector<int> data = {0};
  robot.XPRWModbusRTUReg(slave_id, 0x04, reg_addr, "int16", 1, data, false, ec);
  if (ec) {
    std::cerr << "WARN " << ctx << ":" << ec.message() << std::endl;
    return false;
  }
  value = data[0];
  return true;
}

void init_rs485_gripper(xMateErProRobot &robot, const Config &cfg) {
  error_code ec;
  robot.setxPanelRS485(xPanelOpt::Vout::supply24v, true, ec);
  if (ec) {
    std::cerr << "WARN setxPanelRS485:" << ec.message() << std::endl;
    return;
  }

  if (cfg.gripper_rs485_enable_on_start) {
    // Force serial communication mode (Jodell: 0x03FD = 0x00 for serial mode).
    write_modbus_reg(
      robot,
      cfg.gripper_rs485_slave_id,
      cfg.gripper_rs485_torque_reg,
      0x0000,
      "epg_switch_serial_mode");

    write_modbus_reg(
        robot,
        cfg.gripper_rs485_slave_id,
        cfg.gripper_rs485_init_reg,
        cfg.gripper_rs485_init_value,
        "epg_init");
  }
}

void set_gripper(xMateErProRobot &robot, const Config &cfg, double value) {
  if (!cfg.enable_gripper) {
    return;
  }

  if (cfg.gripper_backend == GripperBackend::Rs485Epg) {
    // Keep tidybot convention: 1.0=open, 0.0=close.
    const double ratio = std::clamp(value, 0.0, 1.0);
    const int target_pos = std::clamp(static_cast<int>(
      std::round(cfg.gripper_rs485_close_pos + ratio * (cfg.gripper_rs485_open_pos - cfg.gripper_rs485_close_pos))), 0, 255);
    const int speed = std::clamp(cfg.gripper_rs485_speed, 0, 255);
    const int torque = std::clamp(cfg.gripper_rs485_torque, 0, 255);

    // Jodell EPG runWithParam protocol:
    // write 3 regs from 0x03E8: [0x0009, pos<<8, speed | (torque<<8)]
    std::vector<int> cmd = {
      to_signed_int16_word(0x0009),
      to_signed_int16_word((target_pos & 0xFF) << 8),
      to_signed_int16_word((speed & 0xFF) | ((torque & 0xFF) << 8)),
    };
    write_modbus_regs(robot, cfg.gripper_rs485_slave_id, cfg.gripper_rs485_pos_reg, cmd, "epg_run_with_param");
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
    else if (arg == "--gripper-backend") {
      const auto backend = need_val(arg);
      if (backend == "di") cfg.gripper_backend = GripperBackend::Di;
      else if (backend == "rs485_epg") cfg.gripper_backend = GripperBackend::Rs485Epg;
      else throw std::runtime_error("unsupported --gripper-backend: " + backend);
    }
    else if (arg == "--gripper-threshold") cfg.gripper_threshold = std::stod(need_val(arg));
    else if (arg == "--gripper-board") cfg.gripper_board = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di1-port") cfg.gripper_di1_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-di2-port") cfg.gripper_di2_port = static_cast<unsigned int>(std::stoul(need_val(arg)));
    else if (arg == "--gripper-rs485-slave-id") cfg.gripper_rs485_slave_id = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-enable-on-start") cfg.gripper_rs485_enable_on_start = true;
    else if (arg == "--gripper-rs485-init-reg") cfg.gripper_rs485_init_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-init-value") cfg.gripper_rs485_init_value = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-torque-reg") cfg.gripper_rs485_torque_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-pos-reg") cfg.gripper_rs485_pos_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-speed-reg") cfg.gripper_rs485_speed_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-pos-now-reg") cfg.gripper_rs485_pos_now_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-force-now-reg") cfg.gripper_rs485_force_now_reg = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-open-pos") cfg.gripper_rs485_open_pos = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-close-pos") cfg.gripper_rs485_close_pos = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-speed") cfg.gripper_rs485_speed = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-torque") cfg.gripper_rs485_torque = parse_int_auto_base(need_val(arg));
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
    else if (arg == "--joint-impedance") {
      if (!parse_csv7(need_val(arg), cfg.joint_impedance)) {
        throw std::runtime_error("bad value for --joint-impedance, expected 7 comma-separated numbers");
      }
    }
    else if (arg == "--max-joint-speed") cfg.max_joint_speed = std::stod(need_val(arg));
    else if (arg == "--max-joint-accel") cfg.max_joint_accel = std::stod(need_val(arg));
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

std::array<double, 16> transform_to_array(const Eigen::Transform<double, 3, Eigen::Isometry> &tf) {
  std::array<double, 16> tf_array{};
  const auto &mat = tf.matrix();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      tf_array[4 * r + c] = mat(r, c);
    }
  }
  return tf_array;
}

void drain_robot_state(xMateErProRobot &robot) {
  while (robot.updateRobotState(std::chrono::steady_clock::duration::zero())) {
  }
}

bool read_current_posture(xMateErProRobot &robot, std::array<double, 6> &posture) {
  try {
    drain_robot_state(robot);
    std::array<double, 16> measured{};
    if (robot.getStateData(RtSupportedFields::tcpPose_m, measured) != 0) {
      return false;
    }
    Utils::transArrayToPosture(measured, posture);
    return true;
  } catch (...) {
    return false;
  }
}

bool read_current_pose_matrix(xMateErProRobot &robot, std::array<double, 16> &pose) {
  try {
    drain_robot_state(robot);
    if (robot.getStateData(RtSupportedFields::tcpPose_m, pose) != 0) {
      return false;
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool read_current_joints(xMateErProRobot &robot, std::array<double, 7> &joints) {
  try {
    drain_robot_state(robot);
    if (robot.getStateData(RtSupportedFields::jointPos_m, joints) != 0) {
      return false;
    }
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

  if (cfg.enable_gripper && cfg.gripper_backend == GripperBackend::Rs485Epg) {
    init_rs485_gripper(robot, cfg);
  }

  try {
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m, RtSupportedFields::jointPos_m});
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

  const auto tf_f2t = flange_to_tool_tf(cfg.tcp_offset_z);
  const auto tf_t2f = tf_f2t.inverse();

  auto model = robot.model();

  const auto preset_joint_rad = Utils::degToRad(cfg.preset_joints_deg);
  const JointPosition preset_joint_pos(std::vector<double>(preset_joint_rad.begin(), preset_joint_rad.end()));

  std::atomic<bool> rt_loop_running(false);
  std::array<double, 6> current_posture{};
  std::array<double, 7> target_joints = preset_joint_rad;
  if (!read_current_joints(robot, target_joints)) {
    target_joints = preset_joint_rad;
  }
  std::array<double, 7> command_joints = target_joints;
  std::array<double, 7> command_joint_vel{};
  std::mutex state_mutex;
  std::mutex target_mutex;
  std::chrono::steady_clock::time_point last_exec_tp = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_callback_tp = std::chrono::steady_clock::now();

  auto configure_rt_joint_impedance = [&]() -> bool {
    robot.setRtNetworkTolerance(40.0, ec);
    warn_ec(ec, "setRtNetworkTolerance");
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (!check_ec(ec, "setMotionControlMode(RtCommand)")) return false;
    robot.setOperateMode(OperateMode::automatic, ec);
    if (!check_ec(ec, "setOperateMode")) return false;
    robot.setPowerState(true, ec);
    if (!check_ec(ec, "setPowerState")) return false;

    rtCon->setFilterLimit(true, cfg.filter_freq);
    rtCon->setJointImpedance(cfg.joint_impedance, ec);
    if (!check_ec(ec, "setJointImpedance")) return false;
    return true;
  };

  std::function<JointPosition(void)> callback = [&]() -> JointPosition {
    std::lock_guard<std::mutex> lock(target_mutex);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_callback_tp).count();
    if (dt <= 0.0 || dt > 0.02) {
      dt = 0.001;
    }
    last_callback_tp = now;

    const double max_speed = std::max(0.0, cfg.max_joint_speed);
    const double max_dv = std::max(0.0, cfg.max_joint_accel) * dt;
    for (size_t i = 0; i < command_joints.size(); ++i) {
      const double delta = target_joints[i] - command_joints[i];
      if (std::abs(delta) <= 1e-9) {
        command_joints[i] = target_joints[i];
        command_joint_vel[i] = 0.0;
        continue;
      }

      double desired_vel = std::clamp(delta / dt, -max_speed, max_speed);
      double dv = desired_vel - command_joint_vel[i];
      dv = std::clamp(dv, -max_dv, max_dv);
      command_joint_vel[i] += dv;

      const double step = command_joint_vel[i] * dt;
      if (std::abs(step) >= std::abs(delta) && step * delta > 0.0) {
        command_joints[i] = target_joints[i];
        command_joint_vel[i] = 0.0;
      } else {
        command_joints[i] += step;
      }
    }

    return JointPosition(std::vector<double>(command_joints.begin(), command_joints.end()));
  };

  auto set_target_joints = [&](const std::array<double, 7> &joints, bool reset_command = false) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target_joints = joints;
    if (reset_command) {
      command_joints = joints;
      command_joint_vel.fill(0.0);
      last_callback_tp = std::chrono::steady_clock::now();
    }
  };

  auto stop_rt_loop = [&]() {
    if (!rt_loop_running.load()) {
      return;
    }
    try {
      rtCon->stopLoop();
    } catch (const std::exception &e) {
      std::cerr << "WARN stopLoop:" << e.what() << std::endl;
    }
    try {
      rtCon->stopMove();
    } catch (const std::exception &e) {
      std::cerr << "WARN stopMove:" << e.what() << std::endl;
    }
    rt_loop_running.store(false);
  };

  auto start_rt_loop = [&]() -> bool {
    if (rt_loop_running.load()) {
      stop_rt_loop();
    }
    try {
      rtCon->setControlLoop(callback);
      rtCon->startMove(RtControllerMode::jointImpedance);
      rtCon->startLoop(false);
      rt_loop_running.store(true);
      return true;
    } catch (const std::exception &e) {
      print_err(std::string("rt_start:") + e.what());
      rt_loop_running.store(false);
      return false;
    }
  };

  if (!configure_rt_joint_impedance()) {
    return 1;
  }
  if (!start_rt_loop()) {
    return 1;
  }

  std::cout << "READY" << std::endl;

  auto restart_rt_from_current = [&]() -> bool {
    std::array<double, 7> joints{};
    if (!read_current_joints(robot, joints)) {
      print_err("read_current_joints");
      return false;
    }
    set_target_joints(joints, true);
    if (!configure_rt_joint_impedance()) {
      return false;
    }
    return start_rt_loop();
  };

  auto execute_nrt_movel = [&](const Eigen::Transform<double, 3, Eigen::Isometry> &target_flange_tf,
                               double speed, double zone) -> bool {
    stop_rt_loop();

    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    if (!check_ec(ec, "setMotionControlMode(NrtCommand)")) {
      restart_rt_from_current();
      return false;
    }
    robot.setOperateMode(OperateMode::automatic, ec);
    if (!check_ec(ec, "setOperateMode")) {
      restart_rt_from_current();
      return false;
    }
    robot.setPowerState(true, ec);
    if (!check_ec(ec, "setPowerState")) {
      restart_rt_from_current();
      return false;
    }

    const auto target_posture = transform_to_posture(target_flange_tf);
    MoveLCommand move_l(
      CartesianPosition{
        target_posture[0], target_posture[1], target_posture[2],
        target_posture[3], target_posture[4], target_posture[5],
      },
      speed,
      zone);
    robot.executeCommand({move_l}, ec);
    if (!check_ec(ec, "executeCommand(MoveLCommand)")) {
      restart_rt_from_current();
      return false;
    }
    if (!wait_robot_idle(robot, std::chrono::seconds(30))) {
      print_err("wait_robot_idle_timeout");
      restart_rt_from_current();
      return false;
    }

    return restart_rt_from_current();
  };

  std::string line;
  double gripper_pos = 1.0;
  double gripper_force = 0.0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;

    if (read_current_posture(robot, current_posture)) {
      std::lock_guard<std::mutex> lock(state_mutex);
    }

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "RESET") {
      stop_rt_loop();

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

      set_target_joints(preset_joint_rad, true);
      if (!configure_rt_joint_impedance()) {
        continue;
      }
      if (!start_rt_loop()) {
        continue;
      }

      last_exec_tp = std::chrono::steady_clock::now();
      gripper_pos = 1.0;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "PRESET") {
      set_target_joints(preset_joint_rad);
      last_exec_tp = std::chrono::steady_clock::now();
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "EXEC") {
      print_err("exec_cartesian_unsupported_in_joint_impedance");
      continue;
    }

    if (cmd == "EXECJ") {
      std::array<double, 7> target_joints{};
      for (double &joint : target_joints) {
        if (!(iss >> joint)) {
          print_err("bad_execj_args");
          target_joints = {};
          break;
        }
      }
      if (!iss) {
        continue;
      }
      double grip = 0.0;
      if (!(iss >> grip)) {
        print_err("bad_execj_args");
        continue;
      }

      set_target_joints(target_joints);

      last_exec_tp = std::chrono::steady_clock::now();
      gripper_pos = grip;
      set_gripper(robot, cfg, gripper_pos);
      std::cout << "OK" << std::endl;
      continue;
    }

    if (cmd == "MOVEL") {
      double x, y, z, qx, qy, qz, qw, grip;
      if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw >> grip)) {
        print_err("bad_movel_args");
        continue;
      }

      double speed = cfg.move_speed;
      double zone = cfg.move_zone;
      iss >> speed >> zone;

      Eigen::Transform<double, 3, Eigen::Isometry> target_tf =
        pose_quat_to_transform(x, y, z, qx, qy, qz, qw);
      target_tf = target_tf * tf_t2f;

      if (!execute_nrt_movel(target_tf, speed, zone)) {
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

      if (cfg.enable_gripper && cfg.gripper_backend == GripperBackend::Rs485Epg) {
        int pos_now_raw = 0;
        if (read_modbus_input_reg(robot, cfg.gripper_rs485_slave_id, cfg.gripper_rs485_pos_now_reg, pos_now_raw, "epg_get_position")) {
          // Jodell reports clamp position in high byte.
          pos_now_raw = (pos_now_raw >> 8) & 0xFF;
          const double denom = static_cast<double>(cfg.gripper_rs485_open_pos - cfg.gripper_rs485_close_pos);
          if (std::abs(denom) > 1e-6) {
            const double ratio = (static_cast<double>(pos_now_raw) - cfg.gripper_rs485_close_pos) / denom;
            gripper_pos = std::clamp(ratio, 0.0, 1.0);
          }
        }

        int force_now_raw = 0;
        if (read_modbus_input_reg(robot, cfg.gripper_rs485_slave_id, cfg.gripper_rs485_force_now_reg, force_now_raw, "epg_get_force")) {
          // Jodell reports clamp force in high byte.
          gripper_force = static_cast<double>((force_now_raw >> 8) & 0xFF);
        }
      }

      std::cout << "STATE "
                << posture[0] << " " << posture[1] << " " << posture[2] << " "
                << posture[3] << " " << posture[4] << " " << posture[5] << " "
                << gripper_pos << " " << gripper_force << std::endl;
      continue;
    }

    if (cmd == "STATEJ") {
      std::array<double, 7> joints{};
      if (!read_current_joints(robot, joints)) {
        print_err("read_current_joints");
        continue;
      }
      std::cout << "STATEJ "
                << joints[0] << " " << joints[1] << " " << joints[2] << " "
                << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << " "
                << gripper_pos << " " << gripper_force << std::endl;
      continue;
    }

    if (cmd == "FKJ") {
      std::array<double, 7> joints{};
      for (double &joint : joints) {
        if (!(iss >> joint)) {
          print_err("bad_fkj_args");
          joints = {};
          break;
        }
      }
      if (!iss) {
        continue;
      }

      const auto flange_tf_arr = model.getCartPose(joints);
      const auto flange_tf = trans_array_to_transform(flange_tf_arr);
      const auto tool_tf = flange_tf * tf_f2t;
      const auto tool_posture = transform_to_posture(tool_tf);
      std::cout << "FKJ "
                << tool_posture[0] << " " << tool_posture[1] << " " << tool_posture[2] << " "
                << tool_posture[3] << " " << tool_posture[4] << " " << tool_posture[5] << std::endl;
      continue;
    }

    if (cmd == "CLOSE") {
      stop_rt_loop();
      std::cout << "OK" << std::endl;
      break;
    }

    print_err("unknown_cmd");
  }

  stop_rt_loop();
  robot.stopReceiveRobotState();
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  warn_ec(ec, "setMotionControlMode(NrtCommand)");

  return 0;
}
