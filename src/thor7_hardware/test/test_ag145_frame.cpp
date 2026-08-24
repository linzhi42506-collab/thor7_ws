// thor7_hardware: Modbus 0x03 响应解析单元测试
//
// 覆盖：正常响应 / CRC 错误 / 错误从站地址 / 错误功能码 / 长度不足 / 异常响应

#include <gtest/gtest.h>

#include <vector>

#include "thor7_hardware/modbus_utils.hpp"

using thor7_hardware::ModbusParseResult;
using thor7_hardware::modbusCrc16;
using thor7_hardware::parseReadHoldingResponse;

namespace {

// 构造带正确 CRC 的响应帧（低字节在前）
std::vector<uint8_t> frameWithCrc(std::vector<uint8_t> body)
{
  const uint16_t crc = modbusCrc16(body);
  body.push_back(static_cast<uint8_t>(crc & 0xFF));
  body.push_back(static_cast<uint8_t>(crc >> 8));
  return body;
}

}  // namespace

TEST(Ag145Frame, NormalReadResponse)
{
  // 位置 1000 = 0x03E8
  const auto frame = frameWithCrc({0x01, 0x03, 0x02, 0x03, 0xE8});
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::OK);
  EXPECT_EQ(value, 1000u);
}

TEST(Ag145Frame, CrcError)
{
  auto frame = frameWithCrc({0x01, 0x03, 0x02, 0x03, 0xE8});
  frame[3] ^= 0xFF;  // 篡改数据字节
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::BAD_CRC);
}

TEST(Ag145Frame, WrongSlaveAddress)
{
  const auto frame = frameWithCrc({0x02, 0x03, 0x02, 0x03, 0xE8});
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::BAD_ADDRESS);
}

TEST(Ag145Frame, WrongFunctionCode)
{
  const auto frame = frameWithCrc({0x01, 0x10, 0x02, 0x03, 0xE8});
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::BAD_FUNCTION);
}

TEST(Ag145Frame, TooShortIncomplete)
{
  const std::vector<uint8_t> frame = {0x01, 0x03};
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::INCOMPLETE);
}

TEST(Ag145Frame, WrongByteCount)
{
  // byte count 错误（应为 0x02）
  const auto frame = frameWithCrc({0x01, 0x03, 0x04, 0x03, 0xE8});
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::BAD_LENGTH);
}

TEST(Ag145Frame, ExceptionResponse)
{
  // 异常响应：功能码 0x03|0x80 = 0x83
  const auto frame = frameWithCrc({0x01, 0x83, 0x02});
  uint16_t value = 0;
  EXPECT_EQ(parseReadHoldingResponse(frame, 0x01, value), ModbusParseResult::EXCEPTION_RESPONSE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
