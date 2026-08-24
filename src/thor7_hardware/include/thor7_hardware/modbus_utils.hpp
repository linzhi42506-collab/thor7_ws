// thor7_hardware: Modbus RTU 纯函数工具（无串口依赖，可单元测试）
//
// AG145 大寰夹爪寄存器约定（只读阶段仅用 0x0200/0x0201/0x0202）：
//   0x0200 初始化反馈 / 0x0201 夹爪状态 / 0x0202 实际位置
// 位置范围 0..1000：0=闭合，1000=张开。
// 与 MoveIt 主控关节的映射（SRDF 约定）：1000 -> -0.65 rad，0 -> 0.0 rad

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace thor7_hardware {

// Modbus CRC16（多项式 0xA001），返回的 16 位值在帧中低字节在前
inline uint16_t modbusCrc16(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// 0x03 读保持寄存器响应帧解析结果
enum class ModbusParseResult {
  OK,                // 解析成功
  BAD_ADDRESS,       // 从站地址不匹配
  BAD_FUNCTION,      // 功能码不匹配且非异常响应
  BAD_LENGTH,        // 长度不足/byte count 不符
  BAD_CRC,           // CRC 校验失败
  EXCEPTION_RESPONSE,// 功能码 | 0x80 的 Modbus 异常响应
  INCOMPLETE,        // 帧不完整（长度不足，无法判断）
};

// 解析 0x03 读保持寄存器（读 1 个寄存器）的响应帧。
// 输入必须为完整帧（含 CRC）。正常响应 7 字节：
//   [slave][0x03][byte_count=0x02][data_hi][data_lo][crc_lo][crc_hi]
// 异常响应 5 字节：
//   [slave][0x83][exception_code][crc_lo][crc_hi]
// 成功时 value_out 为寄存器值。
inline ModbusParseResult parseReadHoldingResponse(
  const std::vector<uint8_t> & frame,
  uint8_t slave_id,
  uint16_t & value_out)
{
  if (frame.size() < 5) {
    return ModbusParseResult::INCOMPLETE;
  }
  if (frame[0] != slave_id) {
    return ModbusParseResult::BAD_ADDRESS;
  }

  const uint8_t func = frame[1];

  // CRC 校验（帧末尾低字节在前：[crc_lo][crc_hi]）
  const uint16_t crc_expected = static_cast<uint16_t>(frame[frame.size() - 2]) |
    (static_cast<uint16_t>(frame[frame.size() - 1]) << 8);
  std::vector<uint8_t> body(frame.begin(), frame.end() - 2);
  if (modbusCrc16(body) != crc_expected) {
    return ModbusParseResult::BAD_CRC;
  }

  // 异常响应：功能码最高位被置 1
  if ((func & 0x80) != 0) {
    return ModbusParseResult::EXCEPTION_RESPONSE;
  }

  if (func != 0x03) {
    return ModbusParseResult::BAD_FUNCTION;
  }

  // 正常响应：读 1 个寄存器 -> byte_count=0x02，总长 7
  if (frame.size() < 7) {
    return ModbusParseResult::BAD_LENGTH;
  }
  if (frame[2] != 0x02) {
    return ModbusParseResult::BAD_LENGTH;
  }
  value_out = static_cast<uint16_t>(frame[3]) << 8 | frame[4];
  return ModbusParseResult::OK;
}

// 设备位置(0..1000) -> MoveIt 主控关节(rad)，映射 -0.65*pos/1000，钳制到 [-0.65, 0.0]
inline double devicePositionToJointRad(int device_position)
{
  const double pos = std::clamp(static_cast<double>(device_position), 0.0, 1000.0);
  return std::clamp(-0.65 * pos / 1000.0, -0.65, 0.0);
}

// MoveIt 主控关节(rad) -> 设备位置(0..1000)，反向映射（仅用于单元测试，本阶段禁止用于发送命令）
inline int jointRadToDevicePosition(double joint_rad)
{
  const double rad = std::clamp(joint_rad, -0.65, 0.0);
  const double pos = (-rad / 0.65) * 1000.0;
  return static_cast<int>(std::lround(pos));
}

// 夹爪状态文本（0 running / 1 stopped / 2 holding / 3 dropped / 4 error / 其它 unknown）
inline const char * gripperStatusText(uint16_t status)
{
  switch (status) {
    case 0: return "running";
    case 1: return "stopped";
    case 2: return "holding";
    case 3: return "dropped";
    case 4: return "error";
    default: return "unknown";
  }
}

}  // namespace thor7_hardware
