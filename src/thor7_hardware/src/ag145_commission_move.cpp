#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "thor7_hardware/ag145_modbus_client.hpp"
#include "thor7_hardware/modbus_utils.hpp"

using thor7_hardware::Ag145ModbusClient;

namespace
{

bool confirm(const std::string & token)
{
  std::cout
    << "\nType exactly [" << token
    << "] and press Enter: "
    << std::flush;

  std::string input;

  if (!std::getline(std::cin, input)) {
    return false;
  }

  return input == token;
}


bool waitForPosition(
  Ag145ModbusClient & client,
  uint16_t target,
  int tolerance_counts,
  double timeout_sec)
{
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(
      static_cast<int>(timeout_sec * 1000.0));

  while (
    std::chrono::steady_clock::now() <
    deadline)
  {
    uint16_t status = 0;
    uint16_t pos = 0;

    if (!client.readStatus(status) ||
        !client.readPosition(pos))
    {
      std::cerr
        << "READ FAIL while waiting for motion\n";
      return false;
    }

    if (status == 4) {
      std::cerr
        << "FAIL: AG145 reports ERROR status=4\n";
      return false;
    }

    const int error =
      std::abs(
        static_cast<int>(pos) -
        static_cast<int>(target));

    std::cout
      << "\r  target=" << target
      << " actual=" << pos
      << " error=" << error
      << " status=" << status
      << "        "
      << std::flush;

    if (
      error <= tolerance_counts &&
      status != 0)
    {
      std::cout << "\n";
      return true;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));
  }

  std::cout << "\n";
  std::cerr << "FAIL: motion timeout\n";
  return false;
}

}  // namespace


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp::Node>(
      "ag145_commission_move");

  const auto port =
    node->declare_parameter<std::string>(
      "serial_port", "/dev/ttyUSB0");

  const int baud =
    node->declare_parameter<int>(
      "baud_rate", 115200);

  const int slave_id =
    node->declare_parameter<int>(
      "slave_id", 1);

  const int timeout_ms =
    node->declare_parameter<int>(
      "timeout_ms", 200);

  const int retries =
    node->declare_parameter<int>(
      "retries", 3);

  Ag145ModbusClient client(
    port,
    static_cast<uint32_t>(baud),
    static_cast<uint8_t>(slave_id),
    timeout_ms,
    retries);

  if (!client.open()) {
    std::cerr
      << "FAIL: cannot open "
      << port << "\n";

    rclcpp::shutdown();
    return 1;
  }

  std::cout
    << "========== AG145 COMMISSION ==========\n"
    << "port      : " << port << "\n"
    << "baud      : " << baud << "\n"
    << "slave     : " << slave_id << "\n";

  uint16_t init = 0;
  uint16_t status = 0;
  uint16_t start = 0;
  uint16_t original_force = 0;

  if (!client.readInitFeedback(init) ||
      !client.readStatus(status) ||
      !client.readPosition(start) ||
      !client.readForce(original_force))
  {
    std::cerr
      << "FAIL: initial Modbus read failed\n";

    rclcpp::shutdown();
    return 2;
  }

  std::cout
    << "\n========== INITIAL STATE ==========\n"
    << "init          = " << init << "\n"
    << "status        = " << status
    << " ("
    << thor7_hardware::gripperStatusText(status)
    << ")\n"
    << "device_pos    = " << start << "\n"
    << "joint_rad     = "
    << thor7_hardware::devicePositionToJointRad(
         start)
    << "\n"
    << "force         = "
    << original_force << " %\n";

  if (init != 1) {
    std::cerr
      << "FAIL: gripper is not initialized\n";

    rclcpp::shutdown();
    return 3;
  }

  if (start > 1000) {
    std::cerr
      << "FAIL: invalid device position\n";

    rclcpp::shutdown();
    return 4;
  }

  if (
    original_force < 20 ||
    original_force > 100)
  {
    std::cerr
      << "FAIL: force register outside "
      << "documented 20..100 range\n";

    rclcpp::shutdown();
    return 5;
  }

  if (status == 4) {
    std::cerr
      << "FAIL: gripper reports ERROR\n";

    rclcpp::shutdown();
    return 6;
  }

  std::cout
    << "\nThis test will:\n"
    << "  1) set temporary force to 20%\n"
    << "  2) write CURRENT position back once\n"
    << "  3) move exactly 200 device counts\n"
    << "  4) return automatically to start\n"
    << "  5) restore original force\n";

  if (!confirm("ARM_AG145")) {
    std::cerr
      << "ABORT: confirmation rejected\n";

    rclcpp::shutdown();
    return 7;
  }

  //
  // Lower force before any physical motion.
  //
  if (!client.writeForce(20)) {
    std::cerr
      << "FAIL: unable to set force=20\n";

    rclcpp::shutdown();
    return 8;
  }

  uint16_t verify_force = 0;

  if (!client.readForce(verify_force) ||
      verify_force != 20)
  {
    std::cerr
      << "FAIL: force write verification failed\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 9;
  }

  std::cout
    << "\nFORCE LIMIT: PASS (20%)\n";

  //
  // ONE zero-motion write.
  //
  std::cout
    << "ZERO-WRITE target = "
    << start << "\n";

  if (!client.writeTargetPosition(start)) {
    std::cerr
      << "FAIL: CURRENT->CURRENT write failed\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 10;
  }

  std::this_thread::sleep_for(
    std::chrono::milliseconds(500));

  uint16_t after_zero = 0;

  if (!client.readPosition(after_zero)) {
    std::cerr
      << "FAIL: zero-write readback failed\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 11;
  }

  const int zero_delta =
    std::abs(
      static_cast<int>(after_zero) -
      static_cast<int>(start));

  std::cout
    << "ZERO-WRITE actual = "
    << after_zero
    << " delta="
    << zero_delta
    << " counts\n";

  if (zero_delta > 10) {
    std::cerr
      << "FAIL: unexpected motion after "
      << "CURRENT->CURRENT\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 12;
  }

  std::cout << "ZERO-WRITE: PASS\n";

  //
  // One meaningful medium move.
  //
  const uint16_t target =
    (start <= 800) ?
    static_cast<uint16_t>(start + 200) :
    static_cast<uint16_t>(start - 200);

  const bool opening = target > start;

  std::cout
    << "\n========== ROUND TRIP ==========\n"
    << "start     = " << start << "\n"
    << "target    = " << target << "\n"
    << "delta     = "
    << (
         static_cast<int>(target) -
         static_cast<int>(start)
       )
    << " counts\n"
    << "direction = "
    << (opening ? "OPEN" : "CLOSE")
    << "\n"
    << "nominal opening change ~= 29 mm\n";

  if (!confirm("MOVE_AG145")) {
    std::cerr
      << "ABORT: motion not authorized\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 13;
  }

  std::cout
    << "\nMOVE TO TARGET...\n";

  if (!client.writeTargetPosition(target)) {
    std::cerr
      << "FAIL: target write failed\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 14;
  }

  if (!waitForPosition(
        client, target, 15, 10.0))
  {
    std::cerr
      << "TARGET FAIL; commanding return "
      << "to start as retreat.\n";

    (void)client.writeTargetPosition(start);
    (void)waitForPosition(
      client, start, 20, 10.0);

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 15;
  }

  std::cout << "TARGET: PASS\n";

  std::this_thread::sleep_for(
    std::chrono::milliseconds(500));

  std::cout
    << "\nRETURN TO START...\n";

  if (!client.writeTargetPosition(start)) {
    std::cerr
      << "FAIL: return command failed\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 16;
  }

  if (!waitForPosition(
        client, start, 15, 10.0))
  {
    std::cerr
      << "FAIL: return did not converge\n";

    (void)client.writeForce(original_force);

    rclcpp::shutdown();
    return 17;
  }

  std::cout << "RETURN: PASS\n";

  //
  // Restore user's original force.
  //
  if (!client.writeForce(original_force)) {
    std::cerr
      << "WARNING: failed to restore "
      << "original force="
      << original_force << "\n";

    rclcpp::shutdown();
    return 18;
  }

  uint16_t final_pos = 0;
  uint16_t final_force = 0;
  uint16_t final_status = 0;

  if (!client.readPosition(final_pos) ||
      !client.readForce(final_force) ||
      !client.readStatus(final_status))
  {
    std::cerr
      << "FAIL: final verification read failed\n";

    rclcpp::shutdown();
    return 19;
  }

  std::cout
    << "\n========== FINAL ==========\n"
    << "position = " << final_pos << "\n"
    << "force    = " << final_force << "\n"
    << "status   = " << final_status
    << " ("
    << thor7_hardware::gripperStatusText(
         final_status)
    << ")\n"
    << "crc_errors = "
    << client.crcErrorCount() << "\n"
    << "timeouts   = "
    << client.timeoutCount() << "\n";

  if (
    std::abs(
      static_cast<int>(final_pos) -
      static_cast<int>(start)) > 15)
  {
    std::cerr
      << "RESULT: FAIL\n";

    rclcpp::shutdown();
    return 20;
  }

  std::cout
    << "\nRESULT: PASS\n";

  client.close();
  rclcpp::shutdown();
  return 0;
}
