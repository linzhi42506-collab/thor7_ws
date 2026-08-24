#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "DianaAPI.h"
#include "thor7_hardware/diana_readonly_client.hpp"

namespace
{

constexpr std::size_t kJointCount = 6;

constexpr double kPi = 3.14159265358979323846;

constexpr double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

constexpr double rad2deg(double rad)
{
  return rad * 180.0 / kPi;
}

// 已确认的 Thor7 安全基准姿态
constexpr std::array<double, kJointCount> kSafePose = {
  deg2rad(107.53),
  deg2rad(-2.78),
  deg2rad(106.76),
  deg2rad(0.48),
  deg2rad(82.30),
  deg2rad(-16.50)
};

// 启动时必须处于安全姿态附近
constexpr double kStartupToleranceRad = deg2rad(3.0);

// 本测试只允许 J2 从当前位置回到 -2.78°。
// 最大移动幅度严格限制在 0.60°。
constexpr double kMaxJ2DeltaRad = deg2rad(0.60);

// servoJ 第一阶段参数
constexpr double kServoTime = 0.020;       // 20 ms
constexpr double kLookAheadTime = 0.100;   // 100 ms
constexpr double kGain = 300.0;

// 100 Hz
constexpr int kControlPeriodMs = 10;

// 用 2.5 秒完成约 0.5° 的超低速运动
constexpr double kRampDurationSec = 2.5;

// 到达最终目标后继续保持 0.5 秒
constexpr double kHoldDurationSec = 0.5;

// 最终位置误差允许 0.10°
constexpr double kFinalToleranceRad = deg2rad(0.10);

std::atomic<bool> g_stop_requested{false};

void signalHandler(int)
{
  // signal handler 内只设置原子变量。
  // 不直接调用 Diana SDK。
  g_stop_requested.store(true);
}

void printJoints(
  const std::string & title,
  const std::array<double, kJointCount> & q)
{
  std::cout << title << "\n";

  for (std::size_t i = 0; i < kJointCount; ++i) {
    std::cout
      << "  J" << i + 1
      << " = "
      << std::fixed << std::setprecision(6)
      << std::setw(10) << q[i]
      << " rad  ("
      << std::setw(9) << std::setprecision(3)
      << rad2deg(q[i])
      << " deg)\n";
  }
}

bool stopRobot(const std::string & ip)
{
  const int ret = ::stop(ip.c_str());

  if (ret < 0) {
    std::cerr
      << "WARNING: stop() returned "
      << ret << "\n";
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  bool execute = false;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--execute") {
      execute = true;
    }
  }

  const std::string robot_ip = "192.168.10.75";

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::cout
    << "\n"
    << "============================================\n"
    << " Thor7 servoJ FIRST CONTROL TEST\n"
    << "============================================\n"
    << "Robot IP       : " << robot_ip << "\n"
    << "Mode           : "
    << (execute ? "REAL EXECUTION" : "DRY-RUN") << "\n"
    << "Target         : J2 -> -2.780 deg\n"
    << "Other joints   : hold current position\n"
    << "Control rate   : 100 Hz\n"
    << "servoJ time    : 0.020 s\n"
    << "look_ahead     : 0.100 s\n"
    << "gain           : 300\n"
    << "Ramp duration  : 2.5 s\n"
    << "Max J2 motion  : 0.60 deg\n\n";

  thor7_hardware::DianaReadonlyClient client(
    robot_ip,
    3,
    1000);

  if (!client.connect()) {
    std::cerr
      << "ERROR: Diana SDK connect failed. code="
      << client.lastSdkError()
      << " message="
      << client.describeError(client.lastSdkError())
      << "\n";

    return 1;
  }

  std::array<double, kJointCount> current{};

  if (!client.readJointPositions(current)) {
    std::cerr
      << "ERROR: initial getJointPos() failed. code="
      << client.lastSdkError()
      << "\n";

    return 1;
  }

  printJoints("Current real position:", current);

  std::cout << "\nStartup safe-pose check:\n";

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double delta =
      std::abs(current[i] - kSafePose[i]);

    std::cout
      << "  J" << i + 1
      << " delta="
      << std::fixed << std::setprecision(3)
      << rad2deg(delta)
      << " deg  ";

    if (delta > kStartupToleranceRad) {
      std::cout << "FAIL\n";
      std::cerr
        << "\nERROR: robot is outside startup safe-pose tolerance.\n";
      return 1;
    }

    std::cout << "PASS\n";
  }

  const char initial_state = client.readRobotState();

  std::cout
    << "\nrobot_state="
    << static_cast<int>(initial_state)
    << "\n";

  if (initial_state != 2) {
    std::cerr
      << "ERROR: robot must be idle (state=2) before servoJ test.\n";
    return 1;
  }

  std::array<double, kJointCount> target = current;

  // 只改变 J2。
  target[1] = kSafePose[1];

  const double j2_delta = target[1] - current[1];

  std::cout << "\n";
  printJoints("Proposed target:", target);

  std::cout
    << "\nJ2 delta = "
    << std::fixed << std::setprecision(3)
    << rad2deg(j2_delta)
    << " deg\n";

  if (std::abs(j2_delta) > kMaxJ2DeltaRad) {
    std::cerr
      << "ERROR: requested J2 motion exceeds hard limit of 0.60 deg.\n";
    return 1;
  }

  if (std::abs(j2_delta) < deg2rad(0.02)) {
    std::cout
      << "\nRobot is already essentially at the J2 safe target.\n"
      << "Nothing to move.\n";
    return 0;
  }

  if (!execute) {
    std::cout
      << "\nDRY-RUN PASS\n"
      << "No servoJ() call was made.\n"
      << "Physical robot was NOT commanded to move.\n";

    return 0;
  }

  std::cout
    << "\n"
    << "*** REAL servoJ MOTION IS ARMED ***\n"
    << "J2 will move slowly toward -2.780 deg.\n"
    << "Other five joints will hold their measured starting positions.\n"
    << "\n"
    << "Type exactly MOVE and press Enter:\n"
    << "> ";

  std::string confirmation;
  std::getline(std::cin, confirmation);

  if (confirmation != "MOVE") {
    std::cout
      << "Aborted. No servoJ() command was sent.\n";
    return 0;
  }

  const int ramp_steps =
    static_cast<int>(
      kRampDurationSec * 1000.0 /
      static_cast<double>(kControlPeriodMs));

  const int hold_steps =
    static_cast<int>(
      kHoldDurationSec * 1000.0 /
      static_cast<double>(kControlPeriodMs));

  std::cout
    << "\nStarting servoJ stream...\n"
    << "Ramp steps = " << ramp_steps << "\n"
    << "Hold steps = " << hold_steps << "\n\n";

  auto next_time = std::chrono::steady_clock::now();

  for (int step = 1; step <= ramp_steps; ++step) {
    if (g_stop_requested.load()) {
      std::cerr << "\nStop requested.\n";
      stopRobot(robot_ip);
      return 1;
    }

    const double alpha =
      static_cast<double>(step) /
      static_cast<double>(ramp_steps);

    std::array<double, 7> command{
      current[0],
      current[1],
      current[2],
      current[3],
      current[4],
      current[5],
      0.0
    };

    command[1] =
      current[1] + alpha * j2_delta;

    const int ret = servoJ(
      command.data(),
      kServoTime,
      kLookAheadTime,
      kGain,
      robot_ip.c_str());

    if (ret < 0) {
      std::cerr
        << "\nERROR: servoJ() failed at step "
        << step
        << ", return="
        << ret
        << "\n";

      stopRobot(robot_ip);
      return 1;
    }

    if (step == 1 || step % 20 == 0 || step == ramp_steps) {
      std::array<double, kJointCount> actual{};

      if (client.readJointPositions(actual)) {
        std::cout
          << "step="
          << std::setw(3) << step
          << "/"
          << ramp_steps
          << " target_J2="
          << std::fixed << std::setprecision(3)
          << rad2deg(command[1])
          << " deg"
          << " actual_J2="
          << rad2deg(actual[1])
          << " deg"
          << " error="
          << rad2deg(command[1] - actual[1])
          << " deg\n";
      }
    }

    next_time +=
      std::chrono::milliseconds(kControlPeriodMs);

    std::this_thread::sleep_until(next_time);
  }

  // 最终目标保持一段时间，让 servoJ 收敛。
  for (int step = 0; step < hold_steps; ++step) {
    if (g_stop_requested.load()) {
      std::cerr << "\nStop requested.\n";
      stopRobot(robot_ip);
      return 1;
    }

    std::array<double, 7> command{
      target[0],
      target[1],
      target[2],
      target[3],
      target[4],
      target[5],
      0.0
    };

    const int ret = servoJ(
      command.data(),
      kServoTime,
      kLookAheadTime,
      kGain,
      robot_ip.c_str());

    if (ret < 0) {
      std::cerr
        << "\nERROR: servoJ() failed during final hold, return="
        << ret
        << "\n";

      stopRobot(robot_ip);
      return 1;
    }

    next_time +=
      std::chrono::milliseconds(kControlPeriodMs);

    std::this_thread::sleep_until(next_time);
  }

  // 在线控制流结束。
  stopRobot(robot_ip);

  std::this_thread::sleep_for(
    std::chrono::milliseconds(200));

  std::array<double, kJointCount> final_position{};

  if (!client.readJointPositions(final_position)) {
    std::cerr
      << "ERROR: failed to read final position.\n";
    return 1;
  }

  std::cout << "\n";
  printJoints("Final real position:", final_position);

  const double final_error =
    target[1] - final_position[1];

  const char final_state =
    client.readRobotState();

  std::cout
    << "\nFinal J2 error = "
    << std::fixed << std::setprecision(4)
    << rad2deg(final_error)
    << " deg\n"
    << "Final robot_state = "
    << static_cast<int>(final_state)
    << "\n";

  if (std::abs(final_error) > kFinalToleranceRad) {
    std::cerr
      << "\nTEST FAILED: final J2 error exceeds 0.10 deg.\n";
    return 1;
  }

  std::cout
    << "\n"
    << "SUCCESS: servoJ 100 Hz control test passed.\n"
    << "J2 reached the safe target within tolerance.\n";

  return 0;
}
