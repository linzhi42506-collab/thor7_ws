// thor7_hardware: Diana 机械臂只读客户端（第一阶段：只读接入验证）
//
// 本类只允许调用 Diana SDK 的只读函数：
//   initSrvNetInfo / initSrvV2 / getJointCount / getJointPos / getRobotState /
//   getInterpolaterCycleInterval / formatError / getLastError / destroySrv
// 严禁调用任何运动/状态改变接口（releaseBrake / moveJToTarget / servoJ / speedJ 等）。
//
// 所有 SDK 函数均显式传入 IP，不依赖空 IP 默认值。
// SDK 回调线程内不得 sleep、不得阻塞、不得发布 ROS 消息，只保存原子状态。

#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include "DianaAPI.h"
#include "DianaAPIDef.h"

namespace thor7_hardware {

class DianaReadonlyClient
{
public:
  // RAII：构造后需调用 connect()；析构时自动 destroySrv(ip)
  DianaReadonlyClient(std::string ip, int retry_count, int retry_delay_ms);
  ~DianaReadonlyClient();

  DianaReadonlyClient(const DianaReadonlyClient &) = delete;
  DianaReadonlyClient & operator=(const DianaReadonlyClient &) = delete;

  // 连接流程：initSrvNetInfo -> initSrvV2 -> getJointCount 验证 == 6
  // 返回 true 表示连接成功且关节数为 6；失败返回 false 并记录原因。
  bool connect();
  void disconnect();

  bool isConnected() const { return connected_.load(); }
  int jointCount() const { return joint_count_; }
  double controlCycleMs() const { return control_cycle_ms_; }
  const std::string & ip() const { return ip_; }

  // 只读：读取 6 个关节位置（固定顺序 joint1..joint6）。
  // 内部使用 getJointPos()（SDK 缓冲为 7 元素，只取前 6 个）。
  bool readJointPositions(std::array<double, 6> & out);

  // 只读：机械臂状态（getRobotState 返回 char，0..8 见 README）
  char readRobotState();

  // 最近一次 SDK 错误码（由 error 回调原子记录）
  int lastSdkError() const { return last_sdk_error_.load(); }
  // 最近一次 warning 回调错误码
  int lastSdkWarning() const { return last_sdk_warning_.load(); }

  // 用 formatError() 把 SDK 错误码转成可读文本
  std::string describeError(int e) const;

  static constexpr int kExpectedJointCount = 6;

private:
  // 静态回调（SDK 在内部线程调用，不得 sleep/阻塞/发布 ROS）
  static void sErrorCallback(int e, const char * str_ip);
  static void sWarningCallback(int e, const char * str_ip);
  static void sStateCallback(StrRobotStateInfo * info, const char * str_ip);

  std::string ip_;
  int retry_count_;
  int retry_delay_ms_;

  std::atomic<bool> connected_{false};
  std::atomic<int> last_sdk_error_{0};
  std::atomic<int> last_sdk_warning_{0};

  int joint_count_{-1};
  double control_cycle_ms_{0.0};

  // SDK 调用串行化（readJointPositions 等并发调用安全）
  std::mutex api_mutex_;
};

}  // namespace thor7_hardware
