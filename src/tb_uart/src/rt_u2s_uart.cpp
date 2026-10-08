#include "tb_uart/rt_u2s_uart.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace tb {

namespace {

// ---------- 상수 ----------
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;

constexpr uint8_t ID_PROGRAM = 0x00;

constexpr uint8_t ID_VEL_ALL     = 0x10;
constexpr uint8_t ID_VEL0        = 0x11;
constexpr uint8_t ID_VEL1        = 0x12;
constexpr uint8_t ID_VEL_REQ_ALL = 0x20;
constexpr uint8_t ID_VEL_REQ0    = 0x21;
constexpr uint8_t ID_VEL_REQ1    = 0x22;

constexpr uint8_t ID_PSD_ALL     = 0x30;
constexpr uint8_t ID_PSD0        = 0x31;
constexpr uint8_t ID_PSD1        = 0x32;
constexpr uint8_t ID_PSD2        = 0x33;
constexpr uint8_t ID_PSD_REQ_ALL = 0x40;
constexpr uint8_t ID_PSD_REQ0    = 0x41;
constexpr uint8_t ID_PSD_REQ1    = 0x42;
constexpr uint8_t ID_PSD_REQ2    = 0x43;

constexpr uint8_t ID_PUB_VEL = 0x50;
constexpr uint8_t ID_PUB_PSD = 0x51;
constexpr uint8_t ID_FILTER = 0x52;

constexpr uint8_t START_VALUE = 0x00;
constexpr uint8_t END_VALUE   = 0xFF;

// ID별 송신 packet 테이블 (index = motor_id / sensor_id)
constexpr uint8_t VEL_IDS[2]     = {ID_VEL0, ID_VEL1};
constexpr uint8_t VEL_REQ_IDS[2] = {ID_VEL_REQ0, ID_VEL_REQ1};
constexpr uint8_t PSD_IDS[3]     = {ID_PSD0, ID_PSD1, ID_PSD2};
constexpr uint8_t PSD_REQ_IDS[3] = {ID_PSD_REQ0, ID_PSD_REQ1, ID_PSD_REQ2};
constexpr uint8_t ID_ERR_COMMON   = 0x60;
constexpr uint8_t ID_ERR_VELOCITY = 0x61;
constexpr uint8_t ID_ERR_PSD      = 0x62;

// ---------- 유틸 ----------
// 수신 대상 packet의 Data 길이. 수신 대상이 아니면 -1
int rx_data_length(uint8_t id)
{
  switch (id) {
    case ID_PROGRAM: return 1;
    case ID_VEL_ALL: return 4;
    case ID_VEL0:
    case ID_VEL1:    return 2;
    case ID_PSD_ALL: return 6;
    case ID_PSD0:
    case ID_PSD1:
    case ID_PSD2:    return 2;
    case ID_ERR_COMMON:
    case ID_ERR_VELOCITY:
    case ID_ERR_PSD: return 1;
    default:         return -1;
  }
}

uint16_t get_u16(const uint8_t *d) { return static_cast<uint16_t>(d[0] | (d[1] << 8)); }
int16_t  get_i16(const uint8_t *d) { return static_cast<int16_t>(get_u16(d)); }

void put_u16(uint8_t *d, uint16_t v)
{
  d[0] = static_cast<uint8_t>(v & 0xFF);
  d[1] = static_cast<uint8_t>(v >> 8);
}

std::string to_hex(const uint8_t *d, int len)
{
  std::string s;
  char t[4];
  for (int i = 0; i < len; ++i) {
    std::snprintf(t, sizeof(t), "%02X", d[i]);
    if (i) s += ' ';
    s += t;
  }
  return s;
}

// 에러 코드 이름. target: 0 공통, 1 velocity, 2 psd. 허용되지 않은 코드는 nullptr
const char *error_name(int target, uint8_t code)
{
  switch (code) {
    case 0x01: return (target == 0) ? "UNKNOWN_ID" : nullptr;
    case 0x02: return "INVALID_DATA";
    case 0x03: return (target == 0) ? "TIMEOUT" : nullptr;
    case 0x04: return (target == 0) ? "NOT_STARTED" : nullptr;
    case 0x10: return (target == 1 || target == 2) ? "SENSOR_FAULT" : nullptr;
    case 0x11: return (target == 1) ? "MOTOR_FAULT" : nullptr;
    default:   return nullptr;
  }
}

const char *id_name(uint8_t id)
{
  switch (id) {
    case ID_PROGRAM:     return "program start/end";
    case ID_VEL_ALL:     return "velocity0,1";
    case ID_VEL0:        return "velocity0";
    case ID_VEL1:        return "velocity1";
    case ID_VEL_REQ_ALL: return "velocity0,1 request";
    case ID_VEL_REQ0:    return "velocity0 request";
    case ID_VEL_REQ1:    return "velocity1 request";
    case ID_PSD_ALL:     return "psd0,1,2";
    case ID_PSD0:        return "psd0";
    case ID_PSD1:        return "psd1";
    case ID_PSD2:        return "psd2";
    case ID_PSD_REQ_ALL: return "psd0,1,2 request";
    case ID_PSD_REQ0:    return "psd0 request";
    case ID_PSD_REQ1:    return "psd1 request";
    case ID_PSD_REQ2:    return "psd2 request";
    case ID_PUB_VEL:     return "velocity period";
    case ID_PUB_PSD:     return "psd period";
    case ID_FILTER:      return "psd filter size";
    case ID_ERR_COMMON:   return "error common";
    case ID_ERR_VELOCITY: return "error velocity";
    case ID_ERR_PSD:      return "error psd";
    default:             return "unknown";
  }
}

}  // namespace

// ---------- 초기화 ----------
bool RtU2sUart::rt_u2s_init(int channel, unsigned int baudrate)
{
  channel_ = channel;
  state_ = State::H1;
  const bool ok = uart_.uart_init(channel, baudrate);
  log("[UART] init channel " + std::to_string(channel) + " @ " +
      std::to_string(baudrate) + " baud : " + (ok ? "OK" : "FAILED"));
  return ok;
}

void RtU2sUart::rt_u2s_close()
{
  uart_.uart_close(channel_);
  state_ = State::H1;
}

// ---------- 공통 송신 ----------
bool RtU2sUart::send_packet(uint8_t id, const uint8_t *data, int len)
{
  uint8_t buf[3 + 6];
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = id;
  if (len > 0) std::memcpy(&buf[3], data, len);
  const int total = 3 + len;

  const bool ok = (uart_.transmit(channel_, buf, total) == total);
  log(std::string("[UART TX] ") + (ok ? "" : "FAILED ") + id_name(id) +
      " : " + to_hex(buf, total));
  return ok;
}

bool RtU2sUart::send_u16_packet(uint8_t id, uint16_t value)
{
  uint8_t d[2];
  put_u16(d, value);
  return send_packet(id, d, 2);
}

// ---------- 시작/종료 (00) ----------
bool RtU2sUart::send_start()
{
  const uint8_t v = START_VALUE;
  return send_packet(ID_PROGRAM, &v, 1);
}

bool RtU2sUart::send_end()
{
  const uint8_t v = END_VALUE;
  return send_packet(ID_PROGRAM, &v, 1);
}

// ---------- velocity 송신 (10, 11, 12) ----------
bool RtU2sUart::send_velocities(int velocity[])
{
  if (velocity == nullptr) return false;

  uint8_t d[4];
  put_u16(&d[0], static_cast<uint16_t>(static_cast<int16_t>(velocity[0])));
  put_u16(&d[2], static_cast<uint16_t>(static_cast<int16_t>(velocity[1])));
  return send_packet(ID_VEL_ALL, d, 4);
}

bool RtU2sUart::send_velocity(int motor_id, int velocity)
{
  if (motor_id < 0 || motor_id > 1) {
    log("[UART TX] invalid motor_id, not sent: " + std::to_string(motor_id));
    return false;
  }
  return send_u16_packet(VEL_IDS[motor_id],
                         static_cast<uint16_t>(static_cast<int16_t>(velocity)));
}

// ---------- velocity 요청 (20, 21, 22) ----------
bool RtU2sUart::request_velocities()
{
  return send_packet(ID_VEL_REQ_ALL, nullptr, 0);
}

bool RtU2sUart::request_velocity(int motor_id)
{
  if (motor_id < 0 || motor_id > 1) {
    log("[UART TX] invalid motor_id, not sent: " + std::to_string(motor_id));
    return false;
  }
  return send_packet(VEL_REQ_IDS[motor_id], nullptr, 0);
}

// ---------- psd 송신 (30, 31, 32, 33) ----------
bool RtU2sUart::send_psds(int psd[])
{
  if (psd == nullptr) return false;

  uint8_t d[6];
  for (int i = 0; i < 3; ++i) put_u16(&d[i * 2], static_cast<uint16_t>(psd[i]));
  return send_packet(ID_PSD_ALL, d, 6);
}

bool RtU2sUart::send_psd(int sensor_id, int psd)
{
  if (sensor_id < 0 || sensor_id > 2) {
    log("[UART TX] invalid sensor_id, not sent: " + std::to_string(sensor_id));
    return false;
  }
  return send_u16_packet(PSD_IDS[sensor_id], static_cast<uint16_t>(psd));
}

// ---------- psd 요청 (40, 41, 42, 43) ----------
bool RtU2sUart::request_psds()
{
  return send_packet(ID_PSD_REQ_ALL, nullptr, 0);
}

bool RtU2sUart::request_psd(int sensor_id)
{
  if (sensor_id < 0 || sensor_id > 2) {
    log("[UART TX] invalid sensor_id, not sent: " + std::to_string(sensor_id));
    return false;
  }
  return send_packet(PSD_REQ_IDS[sensor_id], nullptr, 0);
}

// ---------- 자동 발행 주기 (50, 51) ----------
bool RtU2sUart::send_velocity_period(uint16_t period_ms)
{
  return send_u16_packet(ID_PUB_VEL, period_ms);
}

bool RtU2sUart::send_psd_period(uint16_t period_ms)
{
  return send_u16_packet(ID_PUB_PSD, period_ms);
}

bool RtU2sUart::send_filter_size(uint16_t size)
{
  return send_u16_packet(ID_FILTER, size);
}

// ---------- 수신 ----------
int RtU2sUart::receive(int timeout_ms)
{
  uint8_t buf[256];
  int n = uart_.receive(channel_, buf, sizeof(buf), timeout_ms);
  for (int i = 0; i < n; ++i) feed(buf[i]);
  return n;
}

void RtU2sUart::feed(uint8_t b)
{
  switch (state_) {
    case State::H1:
      if (b == HEADER1) state_ = State::H2;
      break;

    case State::H2:
      if (b == HEADER2) state_ = State::ID;
      else if (b != HEADER1) state_ = State::H1;   // AA AA 55 재동기
      break;

    case State::ID: {
      int len = rx_data_length(b);
      if (len < 0) {                               // 폐기 후 Header 재탐색
        state_ = (b == HEADER1) ? State::H2 : State::H1;
        break;
      }
      id_ = b;
      len_ = static_cast<uint8_t>(len);
      idx_ = 0;
      raw_[0] = HEADER1;
      raw_[1] = HEADER2;
      raw_[2] = b;
      state_ = State::DATA;
      break;
    }

    case State::DATA:
      raw_[3 + idx_] = b;
      data_[idx_++] = b;
      if (idx_ >= len_) {
        state_ = State::H1;
        handle_packet();
      }
      break;
  }
}

void RtU2sUart::handle_packet()
{
  log(std::string("[UART RX] ") + id_name(id_) + " : " + to_hex(raw_, 3 + len_));

  switch (id_) {
    case ID_PROGRAM: {
      if (data_[0] == START_VALUE) {
        log("[UART RX]   stm32 start");
        if (prog_cb_) prog_cb_(true);
      } else if (data_[0] == END_VALUE) {
        log("[UART RX]   stm32 quit");
        if (prog_cb_) prog_cb_(false);
      } else {
        log("[UART RX] invalid start/end value, discarded: " + to_hex(&data_[0], 1));
      }
      break;
    }

    case ID_ERR_COMMON:
    case ID_ERR_VELOCITY:
    case ID_ERR_PSD: {
      const int target = id_ - ID_ERR_COMMON;   // 0 공통, 1 velocity, 2 psd
      const uint8_t code = data_[0];
      const char *name = error_name(target, code);
      if (name == nullptr) {
        log("[UART RX] invalid error code, discarded: " + to_hex(&data_[0], 1));
        break;
      }
      log(std::string("[UART RX]   error ") + id_name(id_) + " : " + name);
      if (err_cb_) err_cb_(target, code, name);
      break;
    }

    case ID_VEL_ALL: {
      const int16_t v[2] = {get_i16(&data_[0]), get_i16(&data_[2])};
      log("[UART RX]   velocity0 = " + std::to_string(v[0]) +
          ", velocity1 = " + std::to_string(v[1]));
      if (vel_cb_) vel_cb_(v);
      break;
    }

    case ID_PSD_ALL: {
      const uint16_t p[3] = {get_u16(&data_[0]), get_u16(&data_[2]), get_u16(&data_[4])};
      log("[UART RX]   psd0 = " + std::to_string(p[0]) +
          ", psd1 = " + std::to_string(p[1]) +
          ", psd2 = " + std::to_string(p[2]));
      if (psd_cb_) psd_cb_(p);
      break;
    }

        case ID_VEL0:
    case ID_VEL1: {
      const int index = (id_ == ID_VEL0) ? 0 : 1;
      const int16_t v = get_i16(&data_[0]);
      log("[UART RX]   velocity" + std::to_string(index) + " = " + std::to_string(v));
      if (vel_one_cb_) vel_one_cb_(index, v);
      break;
    }

    case ID_PSD0:
    case ID_PSD1:
    case ID_PSD2: {
      const int index = id_ - ID_PSD0;
      const uint16_t p = get_u16(&data_[0]);
      log("[UART RX]   psd" + std::to_string(index) + " = " + std::to_string(p));
      if (psd_one_cb_) psd_one_cb_(index, p);
      break;
    }

    default:
      break;
  }
}

}  // namespace tb