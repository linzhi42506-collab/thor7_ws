// thor7_hardware: AG145 大寰夹爪 Modbus RTU 只读客户端
//
// 第一阶段只允许读取：
//   0x0200 初始化反馈 / 0x0201 夹爪状态 / 0x0202 实际位置
// 写操作仅由显式 commissioning/controller 路径调用；readonly probe 本身仍绝不写寄存器。

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "serial/serial.h"

namespace thor7_hardware {

class Ag145ModbusClient
{
public:
  Ag145ModbusClient(
    std::string port,
    uint32_t baud_rate,
    uint8_t slave_id,
    int timeout_ms,
    int retries);
  ~Ag145ModbusClient();

  Ag145ModbusClient(const Ag145ModbusClient &) = delete;
  Ag145ModbusClient & operator=(const Ag145ModbusClient &) = delete;

  bool open();
  void close();
  bool isOpen() const;

  // 只读：初始化反馈寄存器 0x0200
  bool readInitFeedback(uint16_t & out);
  // 只读：夹爪状态寄存器 0x0201
  bool readStatus(uint16_t & out);
  // 只读：实际位置寄存器 0x0202
  bool readPosition(uint16_t & out);

  // 读取当前设定夹持力 0x0101
  bool readForce(uint16_t & out);

  // 显式写操作。调用者必须负责安全门。
  // force: 20..100 %
  bool writeForce(uint16_t force);

  // target: 0..1000, 0=闭合, 1000=张开
  bool writeTargetPosition(uint16_t target);

  uint64_t crcErrorCount() const { return crc_errors_.load(); }
  uint64_t timeoutCount() const { return timeouts_.load(); }
  bool lastReadOk() const { return last_ok_.load(); }

private:
  // 读保持寄存器（0x03），带重试与严格帧校验
  bool readHoldingRegister(uint16_t reg, uint16_t & out);

  // 写单保持寄存器（0x06），严格验证设备回显。
  bool writeSingleRegister(uint16_t reg, uint16_t value);
  bool writeFrame(const std::vector<uint8_t> & frame);
  // 循环累计完整帧（不假设一次 read 得到完整数据）
  bool readFrame(std::vector<uint8_t> & out, size_t expected_len);

  std::string port_;
  uint32_t baud_rate_;
  uint8_t slave_id_;
  int timeout_ms_;
  int retries_;

  std::shared_ptr<serial::Serial> serial_;
  std::mutex comm_mutex_;
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> timeouts_{0};
  std::atomic<bool> last_ok_{false};
};

}  // namespace thor7_hardware
