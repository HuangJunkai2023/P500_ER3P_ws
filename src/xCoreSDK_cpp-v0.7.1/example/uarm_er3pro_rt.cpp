#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

namespace {

std::atomic_bool g_running{true};

void signal_handler(int) {
  g_running.store(false);
}

double now_sec() {
  using clock = std::chrono::steady_clock;
  static const auto t0 = clock::now();
  return std::chrono::duration<double>(clock::now() - t0).count();
}

struct Config {
  std::string robot_ip = "192.168.0.160";
  std::string local_ip = "";
  std::string uarm_port = "/dev/ttyUSB0";
  int uarm_baud = 115200;
  int uarm_timeout_us = 30000;
  int uarm_command_delay_us = 8000;
  double servo_period_ms = 20.0;
  double status_hz = 10.0;
  double stale_timeout_s = 0.30;
  double max_frame_delta_deg = 25.0;
  double filter_freq = 50.0;
  double servoj_kp = 1.0;
  double move_speed = 100.0;
  double move_zone = 5.0;
  bool dry_run = false;
  bool skip_preset = false;
  bool enable_robot = true;
  double startup_uarm_fresh_timeout_s = 5.0;
  std::array<double, 7> preset_joints_deg = {0.0, 30.0, 0.0, 60.0, 0.0, 90.0, 0.0};
  std::array<double, 7> joint_sign = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_scale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_offset_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 7> joint_min_deg = {-170.0, -120.0, -170.0, -170.0, -170.0, -170.0, -170.0};
  std::array<double, 7> joint_max_deg = {170.0, 120.0, 170.0, 170.0, 170.0, 170.0, 170.0};
  std::array<double, 7> max_speed_deg = {90.0, 90.0, 90.0, 120.0, 120.0, 120.0, 120.0};
  double gripper_open_deg = 270.0;
  double gripper_close_deg = 0.0;
};

template <size_t N>
bool parse_csv(const std::string &text, std::array<double, N> &out) {
  std::stringstream ss(text);
  std::string item;
  std::vector<double> vals;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) vals.push_back(std::stod(item));
  }
  if (vals.size() != N) return false;
  for (size_t i = 0; i < N; ++i) out[i] = vals[i];
  return true;
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
    else if (arg == "--uarm-port") cfg.uarm_port = need_val(arg);
    else if (arg == "--uarm-baud") cfg.uarm_baud = std::stoi(need_val(arg));
    else if (arg == "--read-timeout-us") cfg.uarm_timeout_us = std::stoi(need_val(arg));
    else if (arg == "--uarm-command-delay-us") cfg.uarm_command_delay_us = std::stoi(need_val(arg));
    else if (arg == "--servo-period-ms") cfg.servo_period_ms = std::stod(need_val(arg));
    else if (arg == "--status-hz") cfg.status_hz = std::stod(need_val(arg));
    else if (arg == "--stale-timeout") cfg.stale_timeout_s = std::stod(need_val(arg));
    else if (arg == "--startup-uarm-fresh-timeout") cfg.startup_uarm_fresh_timeout_s = std::stod(need_val(arg));
    else if (arg == "--max-frame-delta-deg") cfg.max_frame_delta_deg = std::stod(need_val(arg));
    else if (arg == "--filter-freq") cfg.filter_freq = std::stod(need_val(arg));
    else if (arg == "--servoj-kp") cfg.servoj_kp = std::stod(need_val(arg));
    else if (arg == "--speed") cfg.move_speed = std::stod(need_val(arg));
    else if (arg == "--zone") cfg.move_zone = std::stod(need_val(arg));
    else if (arg == "--gripper-open-deg") cfg.gripper_open_deg = std::stod(need_val(arg));
    else if (arg == "--gripper-close-deg") cfg.gripper_close_deg = std::stod(need_val(arg));
    else if (arg == "--preset-joints-deg") {
      if (!parse_csv(need_val(arg), cfg.preset_joints_deg)) throw std::runtime_error("bad --preset-joints-deg");
    } else if (arg == "--joint-sign") {
      if (!parse_csv(need_val(arg), cfg.joint_sign)) throw std::runtime_error("bad --joint-sign");
    } else if (arg == "--joint-scale") {
      if (!parse_csv(need_val(arg), cfg.joint_scale)) throw std::runtime_error("bad --joint-scale");
    } else if (arg == "--joint-offset-deg") {
      if (!parse_csv(need_val(arg), cfg.joint_offset_deg)) throw std::runtime_error("bad --joint-offset-deg");
    } else if (arg == "--joint-min-deg") {
      if (!parse_csv(need_val(arg), cfg.joint_min_deg)) throw std::runtime_error("bad --joint-min-deg");
    } else if (arg == "--joint-max-deg") {
      if (!parse_csv(need_val(arg), cfg.joint_max_deg)) throw std::runtime_error("bad --joint-max-deg");
    } else if (arg == "--max-speed-deg") {
      if (!parse_csv(need_val(arg), cfg.max_speed_deg)) throw std::runtime_error("bad --max-speed-deg");
    } else if (arg == "--dry-run") {
      cfg.dry_run = true;
      cfg.enable_robot = false;
    } else if (arg == "--no-robot") {
      cfg.enable_robot = false;
    } else if (arg == "--skip-preset") {
      cfg.skip_preset = true;
    } else {
      return false;
    }
  }
  return true;
}

speed_t baud_to_constant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: throw std::runtime_error("unsupported baudrate: " + std::to_string(baud));
  }
}

class ZhonglinSerial {
 public:
  ZhonglinSerial(const std::string &port, int baud, int timeout_us, int command_delay_us)
      : timeout_us_(timeout_us), command_delay_us_(command_delay_us) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("open serial failed: " + port + ": " + std::strerror(errno));

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&tty);
    const auto baud_const = baud_to_constant(baud);
    cfsetispeed(&tty, baud_const);
    cfsetospeed(&tty, baud_const);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr failed");
    tcflush(fd_, TCIOFLUSH);
  }

  ~ZhonglinSerial() {
    if (fd_ >= 0) close(fd_);
  }

  std::string command(const std::string &cmd) {
    tcflush(fd_, TCIFLUSH);
    const ssize_t n = write(fd_, cmd.data(), cmd.size());
    if (n != static_cast<ssize_t>(cmd.size())) return "";
    tcdrain(fd_);
    if (command_delay_us_ > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(command_delay_us_));
    }

    std::string out;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us_);
    char buf[128];
    while (std::chrono::steady_clock::now() < deadline) {
      const ssize_t r = read(fd_, buf, sizeof(buf));
      if (r > 0) {
        out.append(buf, buf + r);
        if (out.find('!') != std::string::npos || out.find('P') != std::string::npos) {
          const auto p = out.find('P');
          if (p != std::string::npos && out.size() >= p + 5) break;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
    return out;
  }

 private:
  int fd_ = -1;
  int timeout_us_;
  int command_delay_us_;
};

bool pwm_to_angle(const std::string &response, double &angle) {
  static const std::regex re("P(\\d{4})");
  std::smatch m;
  if (!std::regex_search(response, m, re)) return false;
  const int pwm = std::stoi(m[1].str());
  angle = (static_cast<double>(pwm) - 500.0) / 2000.0 * 270.0;
  return true;
}

struct SharedState {
  std::mutex mutex;
  std::array<double, 8> zero_deg{};
  std::array<double, 8> uarm_deg{};
  std::array<double, 7> target_rad{};
  std::array<double, 7> measured_rad{};
  std::array<double, 3> tcp_pos{};
  std::array<double, 4> tcp_quat{0.0, 0.0, 0.0, 1.0};
  double gripper = 1.0;
  double uarm_frame_period_ms = 0.0;
  double last_uarm_time = 0.0;
  uint64_t uarm_frames = 0;
  uint64_t serial_errors = 0;
  bool uarm_ready = false;
};

struct StateSnapshot {
  std::array<double, 8> zero_deg{};
  std::array<double, 8> uarm_deg{};
  std::array<double, 7> target_rad{};
  std::array<double, 7> measured_rad{};
  std::array<double, 3> tcp_pos{};
  std::array<double, 4> tcp_quat{0.0, 0.0, 0.0, 1.0};
  double gripper = 1.0;
  double uarm_frame_period_ms = 0.0;
  double last_uarm_time = 0.0;
  uint64_t uarm_frames = 0;
  uint64_t serial_errors = 0;
  bool uarm_ready = false;
};

std::array<double, 7> deg7_to_rad(const std::array<double, 7> &deg) {
  std::array<double, 7> out{};
  for (size_t i = 0; i < out.size(); ++i) out[i] = deg[i] * M_PI / 180.0;
  return out;
}

double gripper_norm_from_angle(double grip_delta_deg, const Config &cfg, double fallback) {
  const double denom = cfg.gripper_open_deg - cfg.gripper_close_deg;
  if (std::abs(denom) < 1e-9) return fallback;
  return std::clamp((grip_delta_deg - cfg.gripper_close_deg) / denom, 0.0, 1.0);
}

std::array<double, 7> map_target_deg(const std::array<double, 8> &angle_deg,
                                     const std::array<double, 8> &zero_deg,
                                     const Config &cfg) {
  std::array<double, 7> target{};
  for (size_t i = 0; i < 7; ++i) {
    const double delta = angle_deg[i] - zero_deg[i];
    target[i] = cfg.preset_joints_deg[i] + cfg.joint_sign[i] * cfg.joint_scale[i] * delta + cfg.joint_offset_deg[i];
    target[i] = std::clamp(target[i], cfg.joint_min_deg[i], cfg.joint_max_deg[i]);
  }
  return target;
}

void uarm_thread_fn(const Config &cfg, SharedState &state) {
  try {
    ZhonglinSerial serial(cfg.uarm_port, cfg.uarm_baud, cfg.uarm_timeout_us, cfg.uarm_command_delay_us);
    serial.command("#000PVER!");
    serial.command("#000PCSK!");

    std::array<double, 8> zero{};
    std::array<double, 8> last{};
    for (size_t i = 0; i < 8; ++i) {
      std::ostringstream id;
      id << std::setw(3) << std::setfill('0') << i;
      serial.command("#" + id.str() + "PULK!");
      const std::string resp = serial.command("#" + id.str() + "PRAD!");
      double angle = 0.0;
      if (pwm_to_angle(resp, angle)) {
        zero[i] = angle;
        last[i] = angle;
      }
    }
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.zero_deg = zero;
      state.uarm_deg = last;
      state.target_rad = deg7_to_rad(map_target_deg(last, zero, cfg));
      state.gripper = 1.0;
      state.uarm_ready = true;
      state.last_uarm_time = now_sec();
    }

    double prev_frame_time = now_sec();
    while (g_running.load()) {
      std::array<double, 8> angles = last;
      size_t valid_reads = 0;
      uint64_t errors = 0;
      for (size_t i = 0; i < 8; ++i) {
        std::ostringstream cmd;
        cmd << "#" << std::setw(3) << std::setfill('0') << i << "PRAD!";
        const std::string resp = serial.command(cmd.str());
        double angle = 0.0;
        if (!pwm_to_angle(resp, angle)) {
          ++errors;
          continue;
        }
        if (std::abs(angle - last[i]) > cfg.max_frame_delta_deg) {
          ++errors;
          continue;
        }
        angles[i] = angle;
        ++valid_reads;
      }

      const double t = now_sec();
      // Match the original UArm reader behavior: a failed servo read keeps its
      // previous value, but any valid read advances the latest command frame.
      if (valid_reads > 0) {
        auto target_deg = map_target_deg(angles, zero, cfg);
        auto target_rad = deg7_to_rad(target_deg);
        const double grip_delta = angles[7] - zero[7];
        std::lock_guard<std::mutex> lock(state.mutex);
        state.uarm_deg = angles;
        state.target_rad = target_rad;
        state.gripper = gripper_norm_from_angle(grip_delta, cfg, state.gripper);
        state.uarm_frame_period_ms = 1000.0 * (t - prev_frame_time);
        state.last_uarm_time = t;
        state.uarm_frames++;
        prev_frame_time = t;
        last = angles;
      }
      if (errors > 0) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.serial_errors += errors;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "ERR uarm:" << e.what() << std::endl;
    g_running.store(false);
  }
}

std::array<double, 7> slew_limit(const std::array<double, 7> &target,
                                 const std::array<double, 7> &last,
                                 const Config &cfg,
                                 double dt_s) {
  std::array<double, 7> out{};
  for (size_t i = 0; i < 7; ++i) {
    const double max_delta = cfg.max_speed_deg[i] * M_PI / 180.0 * dt_s;
    out[i] = last[i] + std::clamp(target[i] - last[i], -max_delta, max_delta);
  }
  return out;
}

bool wait_robot_idle(xMateErProRobot &robot, std::chrono::milliseconds timeout) {
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    error_code ec;
    const auto st = robot.operationState(ec);
    if (!ec && (st == OperationState::idle || st == OperationState::unknown)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

void update_robot_state_cache(xMateErProRobot &robot, SharedState &state) {
  std::array<double, 7> joints{};
  std::array<double, 16> tcp{};
  bool has_joints = false;
  bool has_tcp = false;
  try {
    robot.getStateData(RtSupportedFields::jointPos_m, joints);
    has_joints = true;
  } catch (...) {
  }
  try {
    robot.getStateData(RtSupportedFields::tcpPose_m, tcp);
    has_tcp = true;
  } catch (...) {
  }

  std::lock_guard<std::mutex> lock(state.mutex);
  if (has_joints) state.measured_rad = joints;
  if (has_tcp) {
    Eigen::Matrix3d rot;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) rot(r, c) = tcp[4 * r + c];
    }
    Eigen::Quaterniond q(rot);
    q.normalize();
    state.tcp_pos = {tcp[3], tcp[7], tcp[11]};
    state.tcp_quat = {q.x(), q.y(), q.z(), q.w()};
  }
}

void print_state(const StateSnapshot &snapshot) {
  std::cout << std::fixed << std::setprecision(9)
            << "STATE " << now_sec();
  for (double v : snapshot.measured_rad) std::cout << " " << v;
  for (double v : snapshot.tcp_pos) std::cout << " " << v;
  for (double v : snapshot.tcp_quat) std::cout << " " << v;
  for (double v : snapshot.target_rad) std::cout << " " << v;
  std::cout << " " << snapshot.gripper;
  for (double v : snapshot.uarm_deg) std::cout << " " << v;
  std::cout << " " << (1000.0 * (now_sec() - snapshot.last_uarm_time))
            << " " << snapshot.uarm_frame_period_ms
            << " " << snapshot.uarm_frames
            << " " << snapshot.serial_errors
            << std::endl;
}

StateSnapshot snapshot_state(SharedState &state) {
  std::lock_guard<std::mutex> lock(state.mutex);
  StateSnapshot snapshot;
  snapshot.zero_deg = state.zero_deg;
  snapshot.uarm_deg = state.uarm_deg;
  snapshot.target_rad = state.target_rad;
  snapshot.measured_rad = state.measured_rad;
  snapshot.tcp_pos = state.tcp_pos;
  snapshot.tcp_quat = state.tcp_quat;
  snapshot.gripper = state.gripper;
  snapshot.uarm_frame_period_ms = state.uarm_frame_period_ms;
  snapshot.last_uarm_time = state.last_uarm_time;
  snapshot.uarm_frames = state.uarm_frames;
  snapshot.serial_errors = state.serial_errors;
  snapshot.uarm_ready = state.uarm_ready;
  return snapshot;
}

bool wait_for_fresh_uarm_frame(SharedState &state, double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (g_running.load() && std::chrono::steady_clock::now() < deadline) {
    const auto snap = snapshot_state(state);
    if (snap.uarm_frames > 0 && (now_sec() - snap.last_uarm_time) <= 0.20) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  Config cfg;
  try {
    if (!parse_args(argc, argv, cfg)) {
      std::cerr << "ERR bad_args" << std::endl;
      return 2;
    }
  } catch (const std::exception &e) {
    std::cerr << "ERR args:" << e.what() << std::endl;
    return 2;
  }

  SharedState state;
  state.target_rad = deg7_to_rad(cfg.preset_joints_deg);
  state.measured_rad = state.target_rad;

  std::thread uarm_thread(uarm_thread_fn, std::cref(cfg), std::ref(state));
  while (g_running.load()) {
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      if (state.uarm_ready) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!g_running.load()) {
    if (uarm_thread.joinable()) uarm_thread.join();
    return 1;
  }

  if (!cfg.enable_robot) {
    std::cout << "READY" << std::endl;
    const auto status_period = std::chrono::duration<double>(1.0 / std::max(cfg.status_hz, 0.1));
    auto next_status = std::chrono::steady_clock::now();
    while (g_running.load()) {
      auto snap = snapshot_state(state);
      snap.measured_rad = snap.target_rad;
      print_state(snap);
      next_status += std::chrono::duration_cast<std::chrono::steady_clock::duration>(status_period);
      std::this_thread::sleep_until(next_status);
    }
    if (uarm_thread.joinable()) uarm_thread.join();
    return 0;
  }

  if (!wait_for_fresh_uarm_frame(state, cfg.startup_uarm_fresh_timeout_s)) {
    std::cerr << "ERR uarm:no fresh UArm frame; check /dev/ttyUSB*, baudrate, and Zhonglin protocol timing" << std::endl;
    g_running.store(false);
    if (uarm_thread.joinable()) uarm_thread.join();
    return 1;
  }

  xMateErProRobot robot;
  error_code ec;
  try {
    if (cfg.local_ip.empty()) robot.connectToRobot(cfg.robot_ip);
    else robot.connectToRobot(cfg.robot_ip, cfg.local_ip);

    robot.setOperateMode(OperateMode::automatic, ec);
    if (ec) throw std::runtime_error("setOperateMode:" + ec.message());
    robot.setPowerState(true, ec);
    if (ec) throw std::runtime_error("setPowerState:" + ec.message());

    if (!cfg.skip_preset) {
      robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
      if (ec) throw std::runtime_error("setMotionControlMode(Nrt):" + ec.message());
      const auto preset_rad = deg7_to_rad(cfg.preset_joints_deg);
      JointPosition preset(std::vector<double>(preset_rad.begin(), preset_rad.end()));
      MoveAbsJCommand move(preset, cfg.move_speed, cfg.move_zone);
      robot.executeCommand({move}, ec);
      if (ec) throw std::runtime_error("execute preset:" + ec.message());
      if (!wait_robot_idle(robot, std::chrono::seconds(30))) throw std::runtime_error("preset timeout");
    }

    robot.setRtNetworkTolerance(40.0, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec) throw std::runtime_error("setMotionControlMode(Rt):" + ec.message());
    robot.setOperateMode(OperateMode::automatic, ec);
    if (ec) throw std::runtime_error("setOperateMode Rt:" + ec.message());
    robot.setPowerState(true, ec);
    if (ec) throw std::runtime_error("setPowerState Rt:" + ec.message());
    robot.clearServoAlarm(ec);

    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m, RtSupportedFields::jointPos_m});
    robot.updateRobotState(std::chrono::milliseconds(50));
    update_robot_state_cache(robot, state);

    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) throw std::runtime_error("getRtMotionController:null");
    rtCon->setFilterLimit(false, cfg.filter_freq);
    const double servoj_period_s = cfg.servo_period_ms / 1000.0;
    bool servoj_enabled = false;
    rtCon->setServoJoint(servoj_period_s, servoj_period_s * 3.0, cfg.servoj_kp, ec);
    if (ec) {
      std::cerr << "WARN setServoJoint:" << ec.message()
                << "; fallback to 1ms jointPosition resend mode" << std::endl;
      ec.clear();
    } else {
      servoj_enabled = true;
    }
    rtCon->startMove(RtControllerMode::jointPosition);

    std::cout << "READY" << std::endl;

    auto last_cmd = snapshot_state(state).measured_rad;
    if (std::all_of(last_cmd.begin(), last_cmd.end(), [](double v) { return std::abs(v) < 1e-12; })) {
      last_cmd = deg7_to_rad(cfg.preset_joints_deg);
    }

    auto next_cmd = std::chrono::steady_clock::now();
    auto next_status = std::chrono::steady_clock::now();
    const double command_period_s = servoj_enabled ? servoj_period_s : 0.001;
    const auto cmd_period = std::chrono::duration<double>(command_period_s);
    const auto status_period = std::chrono::duration<double>(1.0 / std::max(cfg.status_hz, 0.1));

    while (g_running.load()) {
      robot.updateRobotState(std::chrono::milliseconds(1));
      update_robot_state_cache(robot, state);

      auto snap = snapshot_state(state);
      const bool stale = (now_sec() - snap.last_uarm_time) > cfg.stale_timeout_s;
      auto target = stale ? last_cmd : snap.target_rad;
      const double dt = std::max(command_period_s, 0.001);
      auto limited = slew_limit(target, last_cmd, cfg, dt);
      last_cmd = limited;
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.target_rad = limited;
      }
      JointPosition cmd(std::vector<double>(limited.begin(), limited.end()));
      rtCon->sendCommand(cmd);

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_status) {
        print_state(snapshot_state(state));
        next_status += std::chrono::duration_cast<std::chrono::steady_clock::duration>(status_period);
      }
      next_cmd += std::chrono::duration_cast<std::chrono::steady_clock::duration>(cmd_period);
      std::this_thread::sleep_until(next_cmd);
    }

    JointPosition finish(std::vector<double>(last_cmd.begin(), last_cmd.end()));
    finish.setFinished();
    rtCon->sendCommand(finish);
    if (servoj_enabled) {
      rtCon->stopServoJoint();
    }
    robot.stopReceiveRobotState();
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  } catch (const std::exception &e) {
    std::cerr << "ERR robot:" << e.what() << std::endl;
    g_running.store(false);
    if (uarm_thread.joinable()) uarm_thread.join();
    return 1;
  }

  g_running.store(false);
  if (uarm_thread.joinable()) uarm_thread.join();
  return 0;
}
