#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "DianaAPI.h"
#include "DianaAPIDef.h"

namespace
{

constexpr const char * kRobotIp = "192.168.10.75";
constexpr std::size_t kJointCount = 6;

// 用户指定的安全基准姿态，单位 rad
constexpr std::array<double, kJointCount> kSafePose{
  1.876752545,   // J1 107.53 deg
 -0.048520153,   // J2  -2.78 deg
  1.863313509,   // J3 106.76 deg
  0.008377580,   // J4   0.48 deg
  1.436405974,   // J5  82.30 deg
 -0.287979327    // J6 -16.50 deg
};

// 启动时每个关节必须在安全姿态 +/- 3 deg 内
constexpr double kStartupTolerance =
  3.0 * 3.14159265358979323846 / 180.0;

// 第一次真实测试：只让 J2 +0.5 deg
constexpr double kJ2Delta =
  0.5 * 3.14159265358979323846 / 180.0;

// 非常保守的第一次运动参数
constexpr double kVelocity = 0.02;      // rad/s
constexpr double kAcceleration = 0.05;  // rad/s^2

// 到位判断约 0.11 deg
constexpr double kPositionTolerance = 0.002;

// 最大等待时间
constexpr double kTimeoutSeconds = 10.0;

std::atomic<bool> g_stop_requested{false};

void signalHandler(int)
{
  g_stop_requested.store(true);
}

const char * stateText(char state)
{
  switch (state) {
    case 0: return "running";
    case 1: return "paused";
    case 2: return "idle";
    case 3: return "free-driving";
    case 4: return "zero-space-free-driving";
    case 5: return "hold-brake";
    case 6: return "error";
    case 7: return "handling-error";
    case 8: return "rescue";
    default: return "unknown";
  }
}

double radToDeg(double value)
{
  return value * 180.0 / 3.14159265358979323846;
}

void printJoints(const char * title, const double * joints)
{
  std::cout << title << "\n";

  for (std::size_t i = 0; i < kJointCount; ++i) {
    std::cout
      << "  J" << (i + 1)
      << " = "
      << std::setw(10) << std::fixed << std::setprecision(6)
      << joints[i] << " rad"
      << "  ("
      << std::setw(9) << std::setprecision(3)
      << radToDeg(joints[i]) << " deg)"
      << "\n";
  }
}

std::string sdkErrorText(int error)
{
  const char * text = formatError(error, kRobotIp);
  return text ? std::string(text) : std::string("(null)");
}

}  // namespace

int main(int argc, char ** argv)
{
  bool execute = false;

  if (argc == 2 && std::string(argv[1]) == "--execute") {
    execute = true;
  } else if (argc > 1) {
    std::cerr
      << "Usage:\n"
      << "  thor7_single_joint_test            # dry-run only\n"
      << "  thor7_single_joint_test --execute  # allow real J2 +0.5 deg motion\n";
    return 2;
  }

  std::signal(SIGINT, signalHandler);

  std::cout << "\n============================================\n";
  std::cout << " Thor7 SINGLE-JOINT FIRST MOTION TEST\n";
  std::cout << "============================================\n";
  std::cout << "Robot IP : " << kRobotIp << "\n";
  std::cout << "Mode     : "
            << (execute ? "REAL EXECUTION" : "DRY-RUN")
            << "\n";
  std::cout << "Motion   : J2 +0.5 deg only\n";
  std::cout << "Velocity : " << kVelocity << " rad/s\n";
  std::cout << "Accel    : " << kAcceleration << " rad/s^2\n\n";

  // ------------------------------------------------------------
  // Connect
  // ------------------------------------------------------------
  srv_net_st net_info;
  std::memset(&net_info, 0, sizeof(net_info));

  initSrvNetInfo(&net_info);

  std::strncpy(
    net_info.SrvIp,
    kRobotIp,
    sizeof(net_info.SrvIp) - 1);

  net_info.SrvIp[sizeof(net_info.SrvIp) - 1] = '\0';

  const int init_ret =
    initSrvV2(nullptr, nullptr, nullptr, &net_info);

  if (init_ret != 0) {
    std::cerr
      << "ERROR: initSrvV2 failed: "
      << init_ret << " "
      << sdkErrorText(init_ret)
      << "\n";
    return 1;
  }

  auto cleanup = []() {
    destroySrv(kRobotIp);
  };

  // ------------------------------------------------------------
  // Basic hardware validation
  // ------------------------------------------------------------
  const int joint_count = getJointCount(kRobotIp);

  if (joint_count != 6) {
    std::cerr
      << "ERROR: expected 6 joints, SDK reports "
      << joint_count << "\n";
    cleanup();
    return 1;
  }

  double current[7]{};

  int ret = getJointPos(current, kRobotIp);

  if (ret != 0) {
    std::cerr
      << "ERROR: getJointPos failed: "
      << ret << " " << sdkErrorText(ret) << "\n";
    cleanup();
    return 1;
  }

  printJoints("Current real position:", current);

  // ------------------------------------------------------------
  // Safe startup-pose guard
  // ------------------------------------------------------------
  bool safe_pose_ok = true;

  std::cout << "\nStartup safe-pose check:\n";

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double delta =
      std::abs(current[i] - kSafePose[i]);

    std::cout
      << "  J" << (i + 1)
      << " delta="
      << std::fixed << std::setprecision(3)
      << radToDeg(delta)
      << " deg";

    if (delta > kStartupTolerance) {
      std::cout << "  FAIL\n";
      safe_pose_ok = false;
    } else {
      std::cout << "  PASS\n";
    }
  }

  if (!safe_pose_ok) {
    std::cerr
      << "\nERROR: robot is not close enough to the configured "
      << "safe startup pose.\n"
      << "Use the teach pendant to return to the safe pose first.\n";

    cleanup();
    return 1;
  }

  // ------------------------------------------------------------
  // Robot must be idle before we even consider motion.
  // ------------------------------------------------------------
  const char initial_state =
    getRobotState(kRobotIp);

  std::cout
    << "\nrobot_state="
    << static_cast<int>(initial_state)
    << " (" << stateText(initial_state) << ")\n";

  if (initial_state != 2) {
    std::cerr
      << "ERROR: robot must be IDLE (state=2) before this test.\n";

    cleanup();
    return 1;
  }

  const int sdk_error =
    getLastError(kRobotIp);

  if (sdk_error != 0) {
    std::cerr
      << "ERROR: SDK reports error="
      << sdk_error
      << " "
      << sdkErrorText(sdk_error)
      << "\n";

    cleanup();
    return 1;
  }

  // ------------------------------------------------------------
  // Read controller-configured limits.
  // ------------------------------------------------------------
  double min_pos[7]{};
  double max_pos[7]{};

  ret = getJointsPositionRange(
    min_pos,
    max_pos,
    kRobotIp);

  if (ret != 0) {
    std::cerr
      << "ERROR: getJointsPositionRange failed: "
      << ret << " " << sdkErrorText(ret) << "\n";

    cleanup();
    return 1;
  }

  // ------------------------------------------------------------
  // Target starts from REAL current state.
  // Only J2 changes.
  // ------------------------------------------------------------
  double target[7]{};

  for (std::size_t i = 0; i < 7; ++i) {
    target[i] = current[i];
  }

  target[1] += kJ2Delta;

  if (target[1] < min_pos[1] ||
      target[1] > max_pos[1])
  {
    std::cerr
      << "ERROR: J2 target is outside configured controller limits.\n";

    cleanup();
    return 1;
  }

  printJoints("\nProposed target:", target);

  std::cout
    << "\nJ2 delta = "
    << std::fixed << std::setprecision(3)
    << radToDeg(target[1] - current[1])
    << " deg\n";

  // ------------------------------------------------------------
  // Default behavior: stop here.
  // ------------------------------------------------------------
  if (!execute) {
    std::cout
      << "\nDRY-RUN PASS\n"
      << "No moveJToTarget() call was made.\n"
      << "Physical robot was NOT commanded to move.\n";

    cleanup();
    return 0;
  }

  // ------------------------------------------------------------
  // Human confirmation immediately before the real SDK call.
  // ------------------------------------------------------------
  std::cout
    << "\n*** REAL MOTION IS ARMED ***\n"
    << "Requirements before continuing:\n"
    << "  - workspace is clear\n"
    << "  - teach pendant / emergency stop is reachable\n"
    << "  - robot is still idle\n"
    << "  - displayed target is correct\n\n"
    << "Type exactly MOVE and press Enter to execute J2 +0.5 deg:\n> "
    << std::flush;

  std::string confirmation;
  std::getline(std::cin, confirmation);

  if (confirmation != "MOVE") {
    std::cout
      << "Confirmation not received. Exiting without motion.\n";

    cleanup();
    return 0;
  }

  // Recheck immediately before motion.
  ret = getJointPos(current, kRobotIp);

  if (ret != 0) {
    std::cerr << "ERROR: final getJointPos check failed.\n";
    cleanup();
    return 1;
  }

  const char pre_move_state =
    getRobotState(kRobotIp);

  if (pre_move_state != 2) {
    std::cerr
      << "ERROR: robot is no longer idle. "
      << "state="
      << static_cast<int>(pre_move_state)
      << " (" << stateText(pre_move_state) << ")\n";

    cleanup();
    return 1;
  }

  // Rebuild target from the latest real state.
  for (std::size_t i = 0; i < 7; ++i) {
    target[i] = current[i];
  }

  target[1] += kJ2Delta;

  std::cout << "\nCalling moveJToTarget() ONCE...\n";

  ret = moveJToTarget(
    target,
    kVelocity,
    kAcceleration,
    0,
    0.0,
    0.0,
    kRobotIp);

  if (ret != 0) {
    std::cerr
      << "moveJToTarget failed immediately: "
      << ret
      << " "
      << sdkErrorText(ret)
      << "\n";

    // Do not change brake or control mode automatically.
    cleanup();
    return 1;
  }

  std::cout
    << "moveJToTarget accepted by SDK.\n"
    << "Monitoring real feedback...\n";

  // ------------------------------------------------------------
  // Monitor motion.
  // ------------------------------------------------------------
  const auto start =
    std::chrono::steady_clock::now();

  while (true) {
    if (g_stop_requested.load()) {
      std::cerr << "\nCtrl+C requested. Calling stop().\n";
      stop(kRobotIp);
      cleanup();
      return 130;
    }

    double feedback[7]{};

    ret = getJointPos(feedback, kRobotIp);

    if (ret != 0) {
      std::cerr
        << "\nFeedback read failed. Calling stop().\n";

      stop(kRobotIp);
      cleanup();
      return 1;
    }

    const char state =
      getRobotState(kRobotIp);

    const double error =
      std::abs(feedback[1] - target[1]);

    std::cout
      << "\rJ2 actual="
      << std::fixed << std::setprecision(3)
      << radToDeg(feedback[1])
      << " deg"
      << " target="
      << radToDeg(target[1])
      << " deg"
      << " error="
      << radToDeg(error)
      << " deg"
      << " state="
      << static_cast<int>(state)
      << "(" << stateText(state) << ")"
      << std::flush;

    // Expected active state.
    if (state == 0) {
      // running
    }
    // Normal completion.
    else if (state == 2) {
      if (error <= kPositionTolerance) {
        std::cout
          << "\n\nSUCCESS: robot returned to IDLE "
          << "and J2 reached the requested target.\n";

        cleanup();
        return 0;
      }
    }
    // Anything else is not accepted for this first test.
    else {
      std::cerr
        << "\n\nUnexpected robot state during motion: "
        << static_cast<int>(state)
        << " (" << stateText(state) << "). Calling stop().\n";

      stop(kRobotIp);
      cleanup();
      return 1;
    }

    const double elapsed =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();

    if (elapsed > kTimeoutSeconds) {
      std::cerr
        << "\n\nTIMEOUT after "
        << kTimeoutSeconds
        << " seconds. Calling stop().\n";

      stop(kRobotIp);
      cleanup();
      return 1;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));
  }
}
