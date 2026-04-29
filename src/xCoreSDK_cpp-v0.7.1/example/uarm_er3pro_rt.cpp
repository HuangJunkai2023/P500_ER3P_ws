#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
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

enum class GripperBackend {
  Di,
  Rs485Epg,
};

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
  double max_frame_delta_deg = 90.0;
  double filter_freq = 50.0;
  double servoj_kp = 1.0;
  double uarm_deadband_deg = 0.0;
  double uarm_step_deadband_deg = 1.0;
  double uarm_filter_alpha = 0.2;
  int uarm_interp_steps = 5;
  double uarm_interp_hz = 50.0;
  double robot_target_filter_hz = 6.0;
  double move_speed = 100.0;
  double move_zone = 5.0;
  bool dry_run = false;
  bool skip_preset = true;
  bool enable_robot = true;
  double startup_uarm_fresh_timeout_s = 5.0;
  std::array<double, 7> preset_joints_deg = {0.0, 30.0, 0.0, 60.0, 0.0, 90.0, 0.0};
  std::array<double, 7> joint_sign = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_scale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_offset_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 7> joint_min_deg = {-170.0, -120.0, -170.0, -170.0, -170.0, -170.0, -170.0};
  std::array<double, 7> joint_max_deg = {170.0, 120.0, 170.0, 170.0, 170.0, 170.0, 170.0};
  std::array<double, 7> max_speed_deg = {120.0, 120.0, 120.0, 160.0, 160.0, 160.0, 160.0};
  std::array<double, 7> max_accel_deg = {600.0, 600.0, 600.0, 800.0, 800.0, 800.0, 800.0};
  double gripper_open_deg = 270.0;
  double gripper_close_deg = 0.0;
  bool enable_gripper = true;
  GripperBackend gripper_backend = GripperBackend::Rs485Epg;
  double gripper_threshold = 0.5;
  unsigned int gripper_board = 2;
  unsigned int gripper_di1_port = 0;
  unsigned int gripper_di2_port = 1;
  int gripper_rs485_slave_id = 9;
  bool gripper_rs485_enable_on_start = false;
  int gripper_rs485_init_reg = 0x03E8;
  int gripper_rs485_init_value = 0x0001;
  int gripper_rs485_torque_reg = 0x03FD;
  int gripper_rs485_pos_reg = 0x03E8;
  int gripper_rs485_open_pos = 0;
  int gripper_rs485_close_pos = 255;
  int gripper_rs485_speed = 240;
  int gripper_rs485_torque = 100;
  double gripper_min_cmd_interval_s = 0.08;
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

int parse_int_auto_base(const std::string &s) {
  return static_cast<int>(std::stol(s, nullptr, 0));
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
    else if (arg == "--uarm-deadband-deg") cfg.uarm_deadband_deg = std::stod(need_val(arg));
    else if (arg == "--uarm-step-deadband-deg") cfg.uarm_step_deadband_deg = std::stod(need_val(arg));
    else if (arg == "--uarm-filter-alpha") cfg.uarm_filter_alpha = std::stod(need_val(arg));
    else if (arg == "--uarm-interp-steps") cfg.uarm_interp_steps = std::stoi(need_val(arg));
    else if (arg == "--uarm-interp-hz") cfg.uarm_interp_hz = std::stod(need_val(arg));
    else if (arg == "--robot-target-filter-hz") cfg.robot_target_filter_hz = std::stod(need_val(arg));
    else if (arg == "--speed") cfg.move_speed = std::stod(need_val(arg));
    else if (arg == "--zone") cfg.move_zone = std::stod(need_val(arg));
    else if (arg == "--gripper-open-deg") cfg.gripper_open_deg = std::stod(need_val(arg));
    else if (arg == "--gripper-close-deg") cfg.gripper_close_deg = std::stod(need_val(arg));
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
    else if (arg == "--gripper-rs485-open-pos") cfg.gripper_rs485_open_pos = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-close-pos") cfg.gripper_rs485_close_pos = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-speed") cfg.gripper_rs485_speed = parse_int_auto_base(need_val(arg));
    else if (arg == "--gripper-rs485-torque") cfg.gripper_rs485_torque = parse_int_auto_base(need_val(arg));
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
    } else if (arg == "--max-accel-deg") {
      if (!parse_csv(need_val(arg), cfg.max_accel_deg)) throw std::runtime_error("bad --max-accel-deg");
    } else if (arg == "--dry-run") {
      cfg.dry_run = true;
      cfg.enable_robot = false;
    } else if (arg == "--no-robot") {
      cfg.enable_robot = false;
    } else if (arg == "--skip-preset") {
      cfg.skip_preset = true;
    } else if (arg == "--use-preset") {
      cfg.skip_preset = false;
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

void init_gripper(xMateErProRobot &robot, const Config &cfg) {
  if (!cfg.enable_gripper || cfg.gripper_backend != GripperBackend::Rs485Epg) return;

  error_code ec;
  robot.setxPanelRS485(xPanelOpt::Vout::supply24v, true, ec);
  if (ec) {
    std::cerr << "WARN setxPanelRS485:" << ec.message() << std::endl;
  }
  if (!cfg.gripper_rs485_enable_on_start) return;

  write_modbus_reg(robot, cfg.gripper_rs485_slave_id, cfg.gripper_rs485_torque_reg, 0x0000, "epg_switch_serial_mode");
  write_modbus_reg(robot, cfg.gripper_rs485_slave_id, cfg.gripper_rs485_init_reg, cfg.gripper_rs485_init_value, "epg_init");
}

int gripper_norm_to_pos(const Config &cfg, double value) {
  const double ratio = std::clamp(value, 0.0, 1.0);
  return std::clamp(static_cast<int>(
      std::round(cfg.gripper_rs485_close_pos + ratio * (cfg.gripper_rs485_open_pos - cfg.gripper_rs485_close_pos))), 0, 255);
}

void set_gripper(xMateErProRobot &robot, const Config &cfg, double value) {
  if (!cfg.enable_gripper) return;

  if (cfg.gripper_backend == GripperBackend::Rs485Epg) {
    const int target_pos = gripper_norm_to_pos(cfg, value);
    const int speed = std::clamp(cfg.gripper_rs485_speed, 0, 255);
    const int torque = std::clamp(cfg.gripper_rs485_torque, 0, 255);
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

struct SharedState {
  std::mutex mutex;
  std::array<double, 8> zero_deg{};
  std::array<double, 8> uarm_deg{};
  std::array<double, 7> teleop_origin_deg{};
  std::array<double, 7> target_rad{};
  std::array<double, 7> command_rad{};
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
  std::array<double, 7> teleop_origin_deg{};
  std::array<double, 7> target_rad{};
  std::array<double, 7> command_rad{};
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

std::array<double, 7> rad7_to_deg(const std::array<double, 7> &rad) {
  std::array<double, 7> out{};
  for (size_t i = 0; i < out.size(); ++i) out[i] = rad[i] * 180.0 / M_PI;
  return out;
}

double gripper_norm_from_angle(double grip_delta_deg, const Config &cfg, double fallback) {
  const double denom = cfg.gripper_open_deg - cfg.gripper_close_deg;
  if (std::abs(denom) < 1e-9) return fallback;
  return std::clamp((grip_delta_deg - cfg.gripper_close_deg) / denom, 0.0, 1.0);
}

std::array<double, 7> map_target_deg(const std::array<double, 7> &filtered_delta_deg,
                                     const std::array<double, 7> &origin_deg,
                                     const Config &cfg) {
  std::array<double, 7> target{};
  for (size_t i = 0; i < 7; ++i) {
    const double delta = filtered_delta_deg[i];
    target[i] = origin_deg[i] + cfg.joint_sign[i] * cfg.joint_scale[i] * delta + cfg.joint_offset_deg[i];
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
    std::array<double, 7> accepted_delta{};
    std::array<double, 7> filtered_delta{};
    double accepted_grip_delta = 0.0;
    double filtered_grip_delta = 0.0;
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
      state.target_rad = deg7_to_rad(map_target_deg(filtered_delta, state.teleop_origin_deg, cfg));
      state.command_rad = state.target_rad;
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
        angles[i] = angle;
        ++valid_reads;
      }

      // Match the original UArm reader behavior: a failed servo read keeps its
      // previous value, but any valid read advances the latest command frame.
      if (valid_reads > 0) {
        for (size_t i = 0; i < 7; ++i) {
          double raw_delta = angles[i] - zero[i];
          if (std::abs(raw_delta) < cfg.uarm_deadband_deg) {
            raw_delta = 0.0;
          }
          if (std::abs(raw_delta - accepted_delta[i]) > cfg.max_frame_delta_deg) {
            ++errors;
            continue;
          } else if (std::abs(raw_delta - accepted_delta[i]) >= cfg.uarm_step_deadband_deg) {
            accepted_delta[i] = raw_delta;
          }
        }

        const double raw_grip_delta = angles[7] - zero[7];
        if (std::abs(raw_grip_delta - accepted_grip_delta) <= cfg.max_frame_delta_deg &&
            std::abs(raw_grip_delta - accepted_grip_delta) >= cfg.uarm_step_deadband_deg) {
          accepted_grip_delta = raw_grip_delta;
        }

        const double alpha = std::clamp(cfg.uarm_filter_alpha, 0.0, 1.0);
        const int interp_steps = std::max(cfg.uarm_interp_steps, 1);
        const double interp_hz = std::max(cfg.uarm_interp_hz, 1.0);
        const auto interp_period = std::chrono::duration<double>(1.0 / interp_hz);
        for (int step = 0; step < interp_steps && g_running.load(); ++step) {
          for (size_t i = 0; i < 7; ++i) {
            filtered_delta[i] += alpha * (accepted_delta[i] - filtered_delta[i]);
          }
          filtered_grip_delta += alpha * (accepted_grip_delta - filtered_grip_delta);

          std::array<double, 7> origin_deg{};
          {
            std::lock_guard<std::mutex> lock(state.mutex);
            origin_deg = state.teleop_origin_deg;
          }
          auto target_deg = map_target_deg(filtered_delta, origin_deg, cfg);
          auto target_rad = deg7_to_rad(target_deg);
          const double t = now_sec();
          {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.uarm_deg = angles;
            state.target_rad = target_rad;
            state.gripper = gripper_norm_from_angle(filtered_grip_delta, cfg, state.gripper);
            state.uarm_frame_period_ms = 1000.0 * (t - prev_frame_time);
            state.last_uarm_time = t;
            state.uarm_frames++;
          }
          prev_frame_time = t;
          std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::steady_clock::duration>(interp_period));
        }

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

std::array<double, 7> trajectory_step(const std::array<double, 7> &target,
                                      const std::array<double, 7> &last,
                                      std::array<double, 7> &velocity,
                                      const Config &cfg,
                                      double dt_s) {
  std::array<double, 7> out{};
  for (size_t i = 0; i < 7; ++i) {
    const double max_vel = cfg.max_speed_deg[i] * M_PI / 180.0;
    const double max_accel = cfg.max_accel_deg[i] * M_PI / 180.0;
    const double err = target[i] - last[i];
    const double desired_vel = std::clamp(err / std::max(dt_s, 1e-6), -max_vel, max_vel);
    const double max_dv = max_accel * dt_s;
    velocity[i] += std::clamp(desired_vel - velocity[i], -max_dv, max_dv);
    velocity[i] = std::clamp(velocity[i], -max_vel, max_vel);
    out[i] = last[i] + velocity[i] * dt_s;
    if ((target[i] - last[i]) * (target[i] - out[i]) <= 0.0) {
      out[i] = target[i];
      velocity[i] = 0.0;
    }
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
  for (double v : snapshot.command_rad) std::cout << " " << v;
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
  snapshot.teleop_origin_deg = state.teleop_origin_deg;
  snapshot.target_rad = state.target_rad;
  snapshot.command_rad = state.command_rad;
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
  state.teleop_origin_deg = cfg.preset_joints_deg;
  state.target_rad = deg7_to_rad(cfg.preset_joints_deg);
  state.command_rad = state.target_rad;
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

    const auto info = robot.robotInfo(ec);
    if (ec) throw std::runtime_error("robotInfo:" + ec.message());
    std::cerr << "INFO robot:type=" << info.type << " joint_num=" << info.joint_num << std::endl;
    if (info.joint_num != 7) {
      throw std::runtime_error("this bridge is for 7-axis xMateER3 Pro/xMateER7 Pro; connected joint_num="
                               + std::to_string(info.joint_num));
    }

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
      std::lock_guard<std::mutex> lock(state.mutex);
      state.teleop_origin_deg = cfg.preset_joints_deg;
      state.target_rad = preset_rad;
      state.command_rad = preset_rad;
      state.measured_rad = preset_rad;
    } else {
      const auto current_rad = robot.jointPos(ec);
      if (ec) throw std::runtime_error("jointPos startup:" + ec.message());
      std::lock_guard<std::mutex> lock(state.mutex);
      state.teleop_origin_deg = rad7_to_deg(current_rad);
      state.target_rad = current_rad;
      state.command_rad = current_rad;
      state.measured_rad = current_rad;
      std::cerr << "INFO startup: using current ER3Pro joints as UArm teleop origin; pass --use-preset to move to preset first"
                << std::endl;
    }

    robot.setRtNetworkTolerance(40.0, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    if (ec) throw std::runtime_error("setMotionControlMode(Rt):" + ec.message());
    robot.setOperateMode(OperateMode::automatic, ec);
    if (ec) throw std::runtime_error("setOperateMode Rt:" + ec.message());
    robot.setPowerState(true, ec);
    if (ec) throw std::runtime_error("setPowerState Rt:" + ec.message());
    robot.clearServoAlarm(ec);
    init_gripper(robot, cfg);

    robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m, RtSupportedFields::jointPos_m});
    robot.updateRobotState(std::chrono::milliseconds(50));
    update_robot_state_cache(robot, state);

    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) throw std::runtime_error("getRtMotionController:null");
    rtCon->setFilterLimit(false, cfg.filter_freq);

    std::array<double, 7> cmd_rad = snapshot_state(state).measured_rad;
    if (std::all_of(cmd_rad.begin(), cmd_rad.end(), [](double v) { return std::abs(v) < 1e-12; })) {
      cmd_rad = snapshot_state(state).target_rad;
    }
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.command_rad = cmd_rad;
    }
    std::array<double, 7> smooth_target_rad = cmd_rad;
    std::array<double, 7> cmd_velocity{};
    uint64_t callback_ticks = 0;
    std::atomic_bool rt_loop_started{false};

    std::function<JointPosition()> callback = [&]() {
      StateSnapshot snap;
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        snap.target_rad = state.target_rad;
        snap.last_uarm_time = state.last_uarm_time;
      }

      const bool stale = (now_sec() - snap.last_uarm_time) > cfg.stale_timeout_s;
      if (stale || cfg.robot_target_filter_hz <= 0.0) {
        smooth_target_rad = stale ? cmd_rad : snap.target_rad;
      } else {
        const double alpha = 1.0 - std::exp(-2.0 * M_PI * cfg.robot_target_filter_hz * 0.001);
        for (size_t i = 0; i < smooth_target_rad.size(); ++i) {
          smooth_target_rad[i] += alpha * (snap.target_rad[i] - smooth_target_rad[i]);
        }
      }
      cmd_rad = trajectory_step(smooth_target_rad, cmd_rad, cmd_velocity, cfg, 0.001);

      if ((++callback_ticks % 10) == 0) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.command_rad = cmd_rad;
      }

      JointPosition cmd(std::vector<double>(cmd_rad.begin(), cmd_rad.end()));
      if (!g_running.load()) {
        cmd.setFinished();
      }
      return cmd;
    };

    rtCon->setControlLoop(callback);
    rtCon->startMove(RtControllerMode::jointPosition);
    rtCon->startLoop(false);
    rt_loop_started.store(true);

    std::cout << "READY" << std::endl;

    auto next_status = std::chrono::steady_clock::now();
    auto next_gripper = std::chrono::steady_clock::now();
    int last_gripper_cmd_pos = -1000;
    double last_gripper_cmd_value = -1.0;
    const auto status_period = std::chrono::duration<double>(1.0 / std::max(cfg.status_hz, 0.1));

    while (g_running.load()) {
      robot.updateRobotState(std::chrono::milliseconds(1));
      update_robot_state_cache(robot, state);

      auto snap = snapshot_state(state);
      const bool stale = (now_sec() - snap.last_uarm_time) > cfg.stale_timeout_s;
      const auto now = std::chrono::steady_clock::now();
      if (cfg.enable_gripper && now >= next_gripper && !stale) {
        const int target_gripper_pos = gripper_norm_to_pos(cfg, snap.gripper);
        const bool gripper_changed = cfg.gripper_backend == GripperBackend::Rs485Epg
          ? std::abs(target_gripper_pos - last_gripper_cmd_pos) >= 2
          : std::abs(snap.gripper - last_gripper_cmd_value) >= 0.05;
        if (gripper_changed) {
        set_gripper(robot, cfg, snap.gripper);
          last_gripper_cmd_pos = target_gripper_pos;
          last_gripper_cmd_value = snap.gripper;
        }
        next_gripper = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(cfg.gripper_min_cmd_interval_s));
      }
      if (now >= next_status) {
        print_state(snapshot_state(state));
        next_status += std::chrono::duration_cast<std::chrono::steady_clock::duration>(status_period);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    g_running.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (rt_loop_started.load()) {
      try {
        rtCon->stopLoop();
      } catch (const std::exception &e) {
        std::cerr << "WARN stopLoop:" << e.what() << std::endl;
      }
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
