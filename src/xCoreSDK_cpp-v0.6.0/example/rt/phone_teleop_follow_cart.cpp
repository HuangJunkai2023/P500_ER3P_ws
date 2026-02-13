/**
 * @file phone_teleop_follow_cart.cpp
 * @brief Phone WebXR teleop with C++ FollowPosition (ER3Pro supported)
 *
 * Flow:
 * iPhone WebXR -> Python UDP bridge -> this process -> FollowPosition update
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rokae/robot.h"
#include "rokae/utility.h"
#include "../print_helper.hpp"

#ifdef _WIN32
#include <conio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
#endif

using namespace rokae;

namespace {
std::atomic_bool running = true;
std::atomic_bool teleop_enabled = false;
std::atomic_bool phone_pose_ready = false;
std::atomic_bool reset_env_requested = false;
std::atomic_int arm_enabled_count{0};
std::atomic<int64_t> last_phone_packet_ms{0};

std::mutex phone_mtx;
Eigen::Vector3d latest_phone_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond latest_phone_rot = Eigen::Quaterniond::Identity();

constexpr double kMaxDelta = 0.15;  // m
constexpr int kUdpPort = 5566;
constexpr int64_t kPacketTimeoutMs = 120;
// Suppress tiny hand jitter from phone sensing.
constexpr double kPosDeadband = 0.0015;     // m
constexpr double kPosStepLimit = 0.020;     // m per control tick
constexpr double kRotDeadband = 0.0080;     // rad
constexpr double kRotStepLimit = 0.20;      // rad per control tick
constexpr bool kInvertPhoneLocalRoll = true; // Fix left/right tilt direction
constexpr bool kInvertPhoneLocalYaw = true;  // Reverse vertical-axis turning direction

xMateErProRobot robot;

std::array<double, 7u> q_drag_er3pro = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};

struct Packet {
  double px{0}, py{0}, pz{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
  int enabled{0};
  int mode{0};  // 1=arm
  int reset_env{0};
};

bool parsePacket(const std::string &line, Packet &p) {
  std::stringstream ss(line);
  std::string tok;
  std::vector<std::string> fields;
  while (std::getline(ss, tok, ',')) fields.push_back(tok);
  if (fields.size() < 9) return false;

  try {
    p.px = std::stod(fields[0]);
    p.py = std::stod(fields[1]);
    p.pz = std::stod(fields[2]);
    p.qx = std::stod(fields[3]);
    p.qy = std::stod(fields[4]);
    p.qz = std::stod(fields[5]);
    p.qw = std::stod(fields[6]);
    p.enabled = std::stoi(fields[7]);
    p.mode = std::stoi(fields[8]);
    p.reset_env = fields.size() >= 10 ? std::stoi(fields[9]) : 0;
  } catch (...) {
    return false;
  }
  return true;
}

int64_t steadyNowMs() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch()).count();
}

void convertWebxrPose(const Packet &pkt, Eigen::Vector3d &pos, Eigen::Quaterniond &rot) {
  // WebXR: +x right, +y up, +z back
  // Robot: +x forward, +y left, +z up
  pos = Eigen::Vector3d(-pkt.py, pkt.px, -pkt.pz);

  // scipy order in tidybot2: [-qy, qx, -qz, qw] = [x,y,z,w]
  // Eigen ctor: Quaterniond(w,x,y,z)
  rot = Eigen::Quaterniond(pkt.qw, -pkt.qy, pkt.qx, -pkt.qz).normalized();

  const Eigen::Vector3d camera_offset(0.0, 0.02, -0.04);
  pos = pos + rot * camera_offset;
}

void udpReceiverLoop(int port) {
#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
    std::cerr << "WSAStartup failed" << std::endl;
    running = false;
    return;
  }
#endif

  int sockfd = static_cast<int>(socket(AF_INET, SOCK_DGRAM, 0));
  if (sockfd < 0) {
    std::cerr << "Failed to create UDP socket" << std::endl;
    running = false;
#ifdef _WIN32
    WSACleanup();
#endif
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "Failed to bind UDP socket on port " << port << std::endl;
    running = false;
#ifdef _WIN32
    closesocket(sockfd);
    WSACleanup();
#else
    close(sockfd);
#endif
    return;
  }

  // Make recvfrom timeout so shutdown can be responsive.
#ifdef _WIN32
  DWORD timeout_ms = 200;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout_ms), sizeof(timeout_ms));
#else
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 200000;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

  std::cout << "Listening phone UDP on 0.0.0.0:" << port << std::endl;

  char buf[512];
  while (running) {
    sockaddr_in src{};
#ifdef _WIN32
    int srclen = sizeof(src);
    int n = recvfrom(sockfd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&src), &srclen);
#else
    socklen_t srclen = sizeof(src);
    ssize_t n = recvfrom(sockfd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&src), &srclen);
#endif
    if (n <= 0) continue;

    buf[n] = '\0';
    std::string line(buf);
    if (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();

    Packet pkt;
    if (!parsePacket(line, pkt)) continue;

    if (pkt.reset_env == 1) {
      teleop_enabled = false;
      arm_enabled_count = 0;
      reset_env_requested = true;
      std::cout << "Received reset_env request from phone" << std::endl;
      continue;
    }

    if (!(pkt.enabled == 1 && pkt.mode == 1)) {
      teleop_enabled = false;
      arm_enabled_count = 0;
      continue;
    }

    const int enabled_cnt = ++arm_enabled_count;
    if (enabled_cnt <= 2) continue;  // Skip first few frames to avoid touch/pose timing mismatch.

    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    convertWebxrPose(pkt, p, q);

    {
      std::lock_guard<std::mutex> lk(phone_mtx);
      latest_phone_pos = p;
      latest_phone_rot = q;
    }
    last_phone_packet_ms.store(steadyNowMs(), std::memory_order_relaxed);
    phone_pose_ready = true;
    teleop_enabled = true;
  }

#ifdef _WIN32
  closesocket(sockfd);
  WSACleanup();
#else
  close(sockfd);
#endif
}

void updatePose(rokae::FollowPosition<7> &fp,
                const Eigen::Transform<double, 3, Eigen::Isometry> &start_pose) {
  auto rtCon = robot.getRtMotionController().lock();

  // Faster following gain than default sample.
  fp.setScale(1.0);

  Eigen::Transform<double, 3, Eigen::Isometry> target = start_pose;

  bool prev_inited = false;
  bool was_active = false;
  Eigen::Vector3d prev_phone_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond prev_phone_rot = Eigen::Quaterniond::Identity();

  using clock = std::chrono::steady_clock;
  auto next_tick = clock::now();
  auto next_log = clock::now();

  while (running) {
    next_tick += std::chrono::milliseconds(10);  // 100 Hz

    if (reset_env_requested.exchange(false)) {
      target = start_pose;
      prev_inited = false;
      was_active = false;
      fp.update(target);
      std::cout << "Received reset_env, returning to initial pose" << std::endl;
    }

    const int64_t now_ms = steadyNowMs();
    const int64_t last_ms = last_phone_packet_ms.load(std::memory_order_relaxed);
    const bool link_alive = (last_ms > 0) && (now_ms - last_ms <= kPacketTimeoutMs);
    const bool active = teleop_enabled && phone_pose_ready && link_alive;

    if (!active) {
      // Keep publishing current target while inactive so reset target converges.
      fp.update(target);
      if (was_active) std::cout << "Teleop inactive, hold target pose" << std::endl;
      prev_inited = false;
      was_active = false;
      std::this_thread::sleep_until(next_tick);
      continue;
    }
    was_active = true;

    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    {
      std::lock_guard<std::mutex> lk(phone_mtx);
      p = latest_phone_pos;
      q = latest_phone_rot;
    }

    if (!prev_inited) {
      prev_phone_pos = p;
      prev_phone_rot = q;
      prev_inited = true;
      std::cout << "Teleop incremental tracking initialized" << std::endl;
      std::this_thread::sleep_until(next_tick);
      continue;
    }

    // Incremental mapping: if phone stops, commanded increment becomes zero.
    Eigen::Vector3d dp = p - prev_phone_pos;
    prev_phone_pos = p;
    double dp_norm = dp.norm();
    if (dp_norm < kPosDeadband) {
      dp.setZero();
    } else if (dp_norm > kPosStepLimit) {
      dp = dp / dp_norm * kPosStepLimit;
    }

    // Use local-frame incremental rotation so phone local axes map naturally to tool local axes.
    Eigen::Quaterniond dq = (prev_phone_rot.inverse() * q).normalized();
    prev_phone_rot = q;
    Eigen::AngleAxisd aa(dq);
    double signed_angle = aa.angle();
    if (signed_angle > M_PI) signed_angle -= 2.0 * M_PI;
    Eigen::Vector3d rotvec = aa.axis() * signed_angle;

    // Empirical correction: reverse left/right phone tilt.
    if (kInvertPhoneLocalRoll) rotvec.x() = -rotvec.x();
    if (kInvertPhoneLocalYaw) rotvec.z() = -rotvec.z();

    double ang = rotvec.norm();
    if (ang < kRotDeadband) {
      dq = Eigen::Quaterniond::Identity();
    } else if (ang > kRotStepLimit) {
      const double ratio = kRotStepLimit / ang;
      rotvec *= ratio;
      const double a = rotvec.norm();
      dq = (a < 1e-9) ? Eigen::Quaterniond::Identity()
                      : Eigen::Quaterniond(Eigen::AngleAxisd(a, rotvec / a)).normalized();
    } else {
      dq = Eigen::Quaterniond(Eigen::AngleAxisd(ang, rotvec / ang)).normalized();
    }

    Eigen::Vector3d new_pos = target.translation() + dp;
    Eigen::Quaterniond new_rot = (Eigen::Quaterniond(target.rotation()) * dq).normalized();

    target = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    target.linear() = new_rot.toRotationMatrix();
    target.translation() = new_pos;

    fp.update(target);
    if (clock::now() >= next_log) {
      const Eigen::Quaterniond qlog(target.rotation());
      std::cout << std::fixed << std::setprecision(4)
                << "[TX] target xyz=("
                << target.translation().x() << ", "
                << target.translation().y() << ", "
                << target.translation().z() << "), quat(wxyz)=("
                << qlog.w() << ", " << qlog.x() << ", " << qlog.y() << ", " << qlog.z() << ")"
                << std::endl;
      next_log += std::chrono::milliseconds(200);  // 5 Hz log output
    }

    if (rtCon->hasMotionError()) {
      print(std::cerr, "Motion error occurred during phone teleop");
      running = false;
      break;
    }

    std::this_thread::sleep_until(next_tick);
  }
}

void teleopPhoneFollow() {
  using namespace rokae::RtSupportedFields;

  error_code ec;

  std::string robot_name = robot.robotInfo(ec).type;
  if (robot_name.find("ER3") == std::string::npos && robot_name.find("xMateErPro3") == std::string::npos) {
    std::cout << "Current robot type: " << robot_name
              << " (example tuned for ER3Pro 7-DoF profile)" << std::endl;
  }

  auto model = robot.model();
  std::shared_ptr<RtMotionControlCobot<7>> rtCon;
  try {
    rtCon = robot.getRtMotionController().lock();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return;
  }

  rtCon->setFilterFrequency(80, 80, 80, ec);
  rtCon->setFilterLimit(true, 60);

  try {
    rtCon->MoveJ(0.4, robot.jointPos(ec), q_drag_er3pro);
  } catch (const std::exception &e) {
    std::cerr << "MoveJ error: " << e.what() << std::endl;
    return;
  }

  auto cart_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  auto quaternion = Utils::eulerToQuaternion(cart_pose.rpy);

  Eigen::Transform<double, 3, Eigen::Isometry> start_pose = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  start_pose.rotate(Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
  start_pose.pretranslate(Eigen::Vector3d(cart_pose.trans[0], cart_pose.trans[1], cart_pose.trans[2]));

  FollowPosition<7> follow_pose(robot, model);

  running = true;
  follow_pose.start(start_pose);

  std::thread udp_thread(udpReceiverLoop, kUdpPort);
  std::thread updater(updatePose, std::ref(follow_pose), std::cref(start_pose));

  print(std::cout, "Phone teleop started");
  print(std::cout, "Run bridge: python3 example/rt/phone_webxr_udp_bridge.py");
  print(std::cout, "Press 'q' to stop");

  std::thread inputer([] {
#ifdef _WIN32
    while (running) {
      if (_kbhit()) {
        const int ch = _getch();
        if (ch == 'q' || ch == 'Q') {
          running = false;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#else
    termios oldt {};
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
      // Fallback: still functional but requires Enter in unusual terminal setups.
      while (running) {
        if (getchar() == 'q') {
          running = false;
          break;
        }
      }
      return;
    }
    termios newt = oldt;
    newt.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    const int oldfl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldfl | O_NONBLOCK);

    while (running) {
      const int ch = getchar();
      if (ch == 'q' || ch == 'Q') {
        running = false;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldfl);
#endif
  });
  inputer.detach();

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  try {
    follow_pose.stop();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  if (udp_thread.joinable()) udp_thread.join();
  if (updater.joinable()) updater.join();
}

}  // namespace

int main() {
  using namespace rokae;

  std::string remoteIP = "192.168.0.160";
  std::string localIP = "192.168.0.200";
  error_code ec;

  try {
    robot.connectToRobot(remoteIP, localIP);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }

  // Clear stale servo alarms / emergency-stop state before enabling motion.
  robot.clearServoAlarm(ec);
  if (ec) {
    print(std::cerr, "clearServoAlarm failed:", ec);
  }
  ec.clear();
  robot.recoverState(1, ec);
  if (ec) {
    print(std::cerr, "recoverState(1) failed:", ec);
  }
  ec.clear();

  robot.setRtNetworkTolerance(60, ec);
  if (ec) {
    print(std::cerr, "setRtNetworkTolerance failed:", ec);
    return 0;
  }
  ec.clear();

  robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
  if (ec) {
    print(std::cerr, "setMotionControlMode(RtCommand) failed:", ec);
    return 0;
  }
  ec.clear();

  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  if (ec) {
    print(std::cerr, "setOperateMode(automatic) failed:", ec);
    return 0;
  }
  ec.clear();

  robot.setPowerState(true, ec);
  if (ec) {
    print(std::cerr, "setPowerState(true) failed:", ec);
    return 0;
  }

  try {
    robot.startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m});
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }

  teleopPhoneFollow();

  robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
  robot.setPowerState(false, ec);
  robot.setOperateMode(rokae::OperateMode::manual, ec);

  return 0;
}
