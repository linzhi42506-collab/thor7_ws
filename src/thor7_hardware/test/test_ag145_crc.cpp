// thor7_hardware: AG145 Modbus CRC16 单元测试
//
// 使用三个固定 Modbus 请求向量验证：
//   - CRC 低字节、高字节与权威参考一致（低字节在前）
//   - 篡改任意字节会导致 CRC 校验失败

#include <gtest/gtest.h>

#include <vector>

#include "thor7_hardware/modbus_utils.hpp"

using thor7_hardware::modbusCrc16;

TEST(Ag145Crc, WriteInitVector)
{
  const std::vector<uint8_t> body = {0x01, 0x06, 0x01, 0x00, 0x00, 0xA5};
  const uint16_t crc = modbusCrc16(body);
  // 权威参考：CRC=0x4D48，低字节在前
  EXPECT_EQ(crc, 0x4D48u);
  EXPECT_EQ(crc & 0xFFu, 0x48u);          // 低字节
  EXPECT_EQ((crc >> 8) & 0xFFu, 0x4Du);   // 高字节
}

TEST(Ag145Crc, ReadPositionVector)
{
  const std::vector<uint8_t> body = {0x01, 0x03, 0x02, 0x02, 0x00, 0x01};
  const uint16_t crc = modbusCrc16(body);
  EXPECT_EQ(crc, 0x7224u);
  EXPECT_EQ(crc & 0xFFu, 0x24u);
  EXPECT_EQ((crc >> 8) & 0xFFu, 0x72u);
}

TEST(Ag145Crc, ReadStatusVector)
{
  const std::vector<uint8_t> body = {0x01, 0x03, 0x02, 0x01, 0x00, 0x01};
  const uint16_t crc = modbusCrc16(body);
  EXPECT_EQ(crc, 0x72D4u);
  EXPECT_EQ(crc & 0xFFu, 0xD4u);
  EXPECT_EQ((crc >> 8) & 0xFFu, 0x72u);
}

TEST(Ag145Crc, TamperAnyByteFailsCrc)
{
  // 篡改读位置请求的每一个字节，CRC 都必须与原始不同
  const std::vector<uint8_t> body = {0x01, 0x03, 0x02, 0x02, 0x00, 0x01};
  const uint16_t original = modbusCrc16(body);
  for (size_t i = 0; i < body.size(); ++i) {
    std::vector<uint8_t> tampered = body;
    tampered[i] = static_cast<uint8_t>(tampered[i] ^ 0xFF);
    EXPECT_NE(modbusCrc16(tampered), original)
      << "tampering byte " << i << " should change CRC";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
