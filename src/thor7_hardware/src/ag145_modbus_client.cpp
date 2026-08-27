// thor7_hardware: AG145 Modbus RTU 只读客户端实现

#include "thor7_hardware/ag145_modbus_client.hpp"
#include "thor7_hardware/modbus_utils.hpp"

#include <chrono>
#include <thread>

namespace thor7_hardware {

namespace {
constexpr uint8_t kReadHoldingRegisters = 0x03;
constexpr uint8_t kWriteSingleRegister = 0x06;

constexpr uint16_t kRegForce = 0x0101;
constexpr uint16_t kRegTargetPosition = 0x0103;

constexpr uint16_t kRegInitFeedback = 0x0200;
constexpr uint16_t kRegStatus = 0x0201;
constexpr uint16_t kRegPosition = 0x0202;
}  // namespace

Ag145ModbusClient::Ag145ModbusClient(
  std::string port, uint32_t baud_rate, uint8_t slave_id, int timeout_ms, int retries)
  : port_(std::move(port)),
    baud_rate_(baud_rate),
    slave_id_(slave_id),
    timeout_ms_(timeout_ms),
    retries_(retries)
{
}

Ag145ModbusClient::~Ag145ModbusClient()
{
  close();
}

bool Ag145ModbusClient::open()
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  try {
    serial_ = std::make_shared<serial::Serial>(
      port_, baud_rate_,
      serial::Timeout::simpleTimeout(timeout_ms_),
      serial::eightbits,
      serial::parity_none,
      serial::stopbits_one,
      serial::flowcontrol_none);
    return serial_->isOpen();
  } catch (const std::exception & e) {
    (void)e;
    return false;
  }
}

void Ag145ModbusClient::close()
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
}

bool Ag145ModbusClient::isOpen() const
{
  return serial_ && serial_->isOpen();
}

bool Ag145ModbusClient::writeFrame(const std::vector<uint8_t> & frame)
{
  if (!isOpen()) {
    return false;
  }
  return serial_->write(frame) == frame.size();
}

bool Ag145ModbusClient::readFrame(std::vector<uint8_t> & out, size_t expected_len)
{
  // Append until the requested TOTAL frame length is available.
  // Do not clear here: readHoldingRegister() first collects the
  // 5-byte minimum/exception frame, then extends the same buffer
  // to the normal 7-byte response length.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms_);
  while (out.size() < expected_len && std::chrono::steady_clock::now() < deadline) {
    std::vector<uint8_t> chunk;
    const size_t got = serial_->read(chunk, expected_len - out.size());
    out.insert(out.end(), chunk.begin(), chunk.end());
    if (got == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return out.size() >= expected_len;
}

bool Ag145ModbusClient::readHoldingRegister(uint16_t reg, uint16_t & out)
{
  std::lock_guard<std::mutex> lock(comm_mutex_);

  for (int attempt = 0; attempt <= retries_; ++attempt) {
    // 请求帧：[slave][0x03][reg_hi][reg_lo][0x00][0x01][crc_lo][crc_hi]
    std::vector<uint8_t> req = {
      slave_id_,
      kReadHoldingRegisters,
      static_cast<uint8_t>(reg >> 8),
      static_cast<uint8_t>(reg & 0xFF),
      0x00,
      0x01,
    };
    const uint16_t crc = modbusCrc16(req);
    req.push_back(static_cast<uint8_t>(crc & 0xFF));
    req.push_back(static_cast<uint8_t>(crc >> 8));

    if (!writeFrame(req)) {
      continue;
    }

    std::vector<uint8_t> resp;
    // 先按异常响应长度(5)读，再按正常响应长度(7)读；循环累计完整帧
    if (!readFrame(resp, 5)) {
      ++timeouts_;
      continue;
    }
    if (resp.size() >= 5 && (resp[1] & 0x80) != 0) {
      // 异常响应（5 字节已完整）
      ++timeouts_;  // 异常视为本次读取失败
      continue;
    }
    if (!readFrame(resp, 7)) {
      ++timeouts_;
      continue;
    }

    uint16_t value = 0;
    const ModbusParseResult r = parseReadHoldingResponse(resp, slave_id_, value);
    if (r == ModbusParseResult::OK) {
      out = value;
      last_ok_.store(true);
      return true;
    }
    if (r == ModbusParseResult::BAD_CRC) {
      ++crc_errors_;
    }
  }
  last_ok_.store(false);
  return false;
}

bool Ag145ModbusClient::writeSingleRegister(
  uint16_t reg, uint16_t value)
{
  std::lock_guard<std::mutex> lock(comm_mutex_);

  for (int attempt = 0; attempt <= retries_; ++attempt) {
    // Modbus 0x06:
    // [slave][06][reg_hi][reg_lo][data_hi][data_lo][crc_lo][crc_hi]
    std::vector<uint8_t> req = {
      slave_id_,
      kWriteSingleRegister,
      static_cast<uint8_t>(reg >> 8),
      static_cast<uint8_t>(reg & 0xFF),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value & 0xFF),
    };

    const uint16_t crc = modbusCrc16(req);
    req.push_back(static_cast<uint8_t>(crc & 0xFF));
    req.push_back(static_cast<uint8_t>(crc >> 8));

    if (!writeFrame(req)) {
      continue;
    }

    std::vector<uint8_t> resp;

    // 异常 Modbus 响应为 5 bytes。
    if (!readFrame(resp, 5)) {
      ++timeouts_;
      continue;
    }

    if ((resp[1] & 0x80) != 0) {
      ++timeouts_;
      continue;
    }

    // 正常 0x06 响应完整回显请求，共 8 bytes。
    if (!readFrame(resp, 8)) {
      ++timeouts_;
      continue;
    }

    if (resp.size() != 8 ||
        resp[0] != slave_id_ ||
        resp[1] != kWriteSingleRegister)
    {
      continue;
    }

    const uint16_t crc_expected =
      static_cast<uint16_t>(resp[6]) |
      (static_cast<uint16_t>(resp[7]) << 8);

    std::vector<uint8_t> body(
      resp.begin(), resp.begin() + 6);

    if (modbusCrc16(body) != crc_expected) {
      ++crc_errors_;
      continue;
    }

    const uint16_t echoed_reg =
      (static_cast<uint16_t>(resp[2]) << 8) |
      resp[3];

    const uint16_t echoed_value =
      (static_cast<uint16_t>(resp[4]) << 8) |
      resp[5];

    if (echoed_reg != reg ||
        echoed_value != value)
    {
      continue;
    }

    last_ok_.store(true);
    return true;
  }

  last_ok_.store(false);
  return false;
}


bool Ag145ModbusClient::readForce(uint16_t & out)
{
  return readHoldingRegister(kRegForce, out);
}


bool Ag145ModbusClient::writeInitialize()
{
  return writeSingleRegister(
    0x0100,
    0x00A5);
}

bool Ag145ModbusClient::writeForce(uint16_t force)
{
  if (force < 20 || force > 100) {
    return false;
  }

  return writeSingleRegister(
    kRegForce, force);
}


bool Ag145ModbusClient::writeTargetPosition(
  uint16_t target)
{
  if (target > 1000) {
    return false;
  }

  return writeSingleRegister(
    kRegTargetPosition, target);
}


bool Ag145ModbusClient::readInitFeedback(uint16_t & out)
{
  return readHoldingRegister(kRegInitFeedback, out);
}

bool Ag145ModbusClient::readStatus(uint16_t & out)
{
  return readHoldingRegister(kRegStatus, out);
}

bool Ag145ModbusClient::readPosition(uint16_t & out)
{
  return readHoldingRegister(kRegPosition, out);
}

}  // namespace thor7_hardware
