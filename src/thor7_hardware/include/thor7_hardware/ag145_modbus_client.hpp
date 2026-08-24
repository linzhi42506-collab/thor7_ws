// thor7_hardware: AG145 大寰夹爪 Modbus RTU 只读客户端
//
// 第一阶段只允许读取：
//   0x0200 初始化反馈 / 0x0201 夹爪状态 / 0x0202 实际位置
// 绝对禁止写任何寄存器（0x0100/0x0101/0x0103 等）与调用 Initialize()。

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

  uint64_t crcErrorCount() const { return crc_errors_.load(); }
  uint64_t timeoutCount() const { return timeouts_.load(); }
  bool lastReadOk() const { return last_ok_.load(); }

private:
  // 读保持寄存器（0x03），带重试与严格帧校验
  bool readHoldingRegister(uint16_t reg, uint16_t & out);
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
