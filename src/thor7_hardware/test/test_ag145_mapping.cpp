// thor7_hardware: AG145 位置映射单元测试
//
// 设备位置(0..1000) <-> MoveIt 主控关节(rad) 的映射与钳制：
//   0 -> 0.0 rad，1000 -> -0.65 rad，500 -> -0.325 rad
// 正反向映射误差不超过一个设备计数。

#include <gtest/gtest.h>

#include <cmath>

#include "thor7_hardware/modbus_utils.hpp"

using thor7_hardware::devicePositionToJointRad;
using thor7_hardware::jointRadToDevicePosition;

TEST(Ag145Mapping, Endpoints)
{
  EXPECT_DOUBLE_EQ(devicePositionToJointRad(0), 0.0);
  EXPECT_NEAR(devicePositionToJointRad(1000), -0.65, 1e-9);
}

TEST(Ag145Mapping, Midpoint)
{
  EXPECT_NEAR(devicePositionToJointRad(500), -0.325, 1e-9);
}

TEST(Ag145Mapping, ClampOutOfRange)
{
  // 越界输入被正确钳制
  EXPECT_DOUBLE_EQ(devicePositionToJointRad(-5), 0.0);        // 负值 -> 0
  EXPECT_NEAR(devicePositionToJointRad(1500), -0.65, 1e-9);   // 超上限 -> -0.65
  // 反向同样钳制
  EXPECT_EQ(jointRadToDevicePosition(0.5), 0);                // 正值 -> 0
  EXPECT_EQ(jointRadToDevicePosition(-1.0), 1000);            // 超下限 -> 1000
}

TEST(Ag145Mapping, RoundTripErrorWithinOneCount)
{
  // 正向->反向->正向 误差不超过一个设备计数
  for (int pos = 0; pos <= 1000; pos += 37) {
    const double rad = devicePositionToJointRad(pos);
    const int back = jointRadToDevicePosition(rad);
    const double rad2 = devicePositionToJointRad(back);
    EXPECT_LE(std::fabs(rad2 - rad), 0.65 / 1000.0 + 1e-9)
      << "round-trip error too large at device position " << pos;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
