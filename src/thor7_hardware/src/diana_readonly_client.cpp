// thor7_hardware: Diana 只读客户端实现（见头文件注释）

#include "thor7_hardware/diana_readonly_client.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace thor7_hardware {

namespace {
// 当前工程明确只允许一个 Diana 真机客户端同时连接。
// SDK callback 没有 user-data 参数，因此在此锁存异步健康状态。
std::atomic<bool> g_sdk_alive{false};
std::atomic<int> g_sdk_callback_error{0};
std::atomic<int> g_sdk_callback_warning{0};
}  // namespace

DianaReadonlyClient::DianaReadonlyClient(std::string ip, int retry_count, int retry_delay_ms)
  : ip_(std::move(ip)), retry_count_(retry_count), retry_delay_ms_(retry_delay_ms)
{
}

DianaReadonlyClient::~DianaReadonlyClient()
{
  disconnect();
}

void DianaReadonlyClient::sErrorCallback(int e, const char * str_ip)
{
  // callback 线程只做 atomic store。
  // 一旦出现异步 SDK error，故障锁存到下一次重新 connect()。
  g_sdk_callback_error.store(e);
  g_sdk_alive.store(false);
  (void)str_ip;
}

void DianaReadonlyClient::sWarningCallback(int e, const char * str_ip)
{
  g_sdk_callback_warning.store(e);
  (void)str_ip;
}

void DianaReadonlyClient::sStateCallback(StrRobotStateInfo * info, const char * str_ip)
{
  // 回调线程：只读取并缓存错误码（原子），不做阻塞操作
  (void)str_ip;
  if (info != nullptr) {
    g_sdk_alive.store(true);
  }
}

bool DianaReadonlyClient::connect()
{
  std::lock_guard<std::mutex> lock(api_mutex_);

  for (int attempt = 0; attempt <= retry_count_; ++attempt) {
    // 1. 清零并初始化网络结构
    srv_net_st net_info;
    std::memset(&net_info, 0, sizeof(net_info));
    initSrvNetInfo(&net_info);

    // 2. 安全写入 IP（srv_net_st::SrvIp 固定 32 字节，用 strncpy 防止越界）
    std::strncpy(net_info.SrvIp, ip_.c_str(), sizeof(net_info.SrvIp) - 1);
    net_info.SrvIp[sizeof(net_info.SrvIp) - 1] = '\0';

    // 3. 建立连接（V2 带 warning 回调）
    const int ret = initSrvV2(&DianaReadonlyClient::sErrorCallback,
                              &DianaReadonlyClient::sWarningCallback,
                              &DianaReadonlyClient::sStateCallback,
                              &net_info);
    if (ret != NO_ERROR_CODE) {
      last_sdk_error_.store(ret);
      if (attempt < retry_count_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms_));
        continue;
      }
      return false;
    }

    // 4. 验证关节数必须为 6
    const int count = getJointCount(ip_.c_str());
    if (count != kExpectedJointCount) {
      last_sdk_error_.store(-1000);  // 自定义：关节数不符
      destroySrv(ip_.c_str());
      return false;
    }
    joint_count_ = count;

    // 5. 读取控制周期（只读）
    double interval = 0.0;
    if (getInterpolaterCycleInterval(interval, ip_.c_str()) == NO_ERROR_CODE) {
      control_cycle_ms_ = interval * 1000.0;
    }

    connected_.store(true);
    g_sdk_callback_error.store(0);
    g_sdk_callback_warning.store(0);
    g_sdk_alive.store(true);
    last_sdk_error_.store(0);
    last_sdk_warning_.store(0);
    return true;
  }
  return false;
}

bool DianaReadonlyClient::isConnected() const
{
  return
    connected_.load() &&
    g_sdk_alive.load() &&
    g_sdk_callback_error.load() == 0;
}

int DianaReadonlyClient::lastSdkError() const
{
  const int callback_error = g_sdk_callback_error.load();

  if (callback_error != 0) {
    return callback_error;
  }

  return last_sdk_error_.load();
}

int DianaReadonlyClient::lastSdkWarning() const
{
  const int callback_warning = g_sdk_callback_warning.load();

  if (callback_warning != 0) {
    return callback_warning;
  }

  return last_sdk_warning_.load();
}

void DianaReadonlyClient::disconnect()
{
  std::lock_guard<std::mutex> lock(api_mutex_);
  if (connected_.exchange(false)) {
    destroySrv(ip_.c_str());
  }
  g_sdk_alive.store(false);
}

bool DianaReadonlyClient::readJointPositions(std::array<double, 6> & out)
{
  if (!connected_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(api_mutex_);

  // SDK 头文件声明 OUT[7]，安全分配 7 个元素，只取前 6 个（Thor 为 6 关节）
  double joints[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const int ret = getJointPos(joints, ip_.c_str());
  if (ret != NO_ERROR_CODE) {
    last_sdk_error_.store(ret);
    return false;
  }
  for (int i = 0; i < 6; ++i) {
    out[static_cast<size_t>(i)] = joints[i];
  }
  return true;
}

char DianaReadonlyClient::readRobotState()
{
  if (!connected_.load()) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(api_mutex_);
  return getRobotState(ip_.c_str());
}

std::string DianaReadonlyClient::describeError(int e) const
{
  const char * msg = formatError(e, ip_.c_str());
  return msg != nullptr ? std::string(msg) : std::string("(null)");
}

}  // namespace thor7_hardware
