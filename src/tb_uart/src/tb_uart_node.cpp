#include "tb_uart/tb_uart_node.hpp"

#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

// ---------- topic 이름 ----------
#define TOPIC_COMMAND "TB_Uart_RX"   // 입력: 명령 수신
#define TOPIC_STATUS  "TB_Uart_TX"    // 출력: 상태/데이터 발행

namespace tb {

namespace {

// 공백 기준으로 단어 분리
std::vector<std::string> split(const std::string &s)
{
  std::istringstream iss(s);
  std::vector<std::string> out;
  std::string t;
  while (iss >> t) out.push_back(t);
  return out;
}

// 문자열 전체가 정수일 때만 true
bool to_int(const std::string &s, int &v)
{
  if (s.empty()) return false;
  char *end = nullptr;
  errno = 0;
  const long r = std::strtol(s.c_str(), &end, 10);
  if (errno != 0 || *end != '\0') return false;
  v = static_cast<int>(r);
  return true;
}

}  // namespace

TbUartNode::TbUartNode() : Node("tb_uart_node")
{
  channel_ = declare_parameter<int>("channel", -1);   // -1: 자동 탐색

  pub_ = create_publisher<std_msgs::msg::String>(TOPIC_STATUS, 10);
  sub_ = create_subscription<std_msgs::msg::String>(
    TOPIC_COMMAND, 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_command(msg); });

  rt_.set_velocity_callback([this](const int16_t v[2]) { on_velocity(v); });
  rt_.set_psd_callback([this](const uint16_t p[3]) { on_psd(p); });
  rt_.set_velocity_one_callback([this](int i, int16_t v) { on_velocity_one(i, v); });
  rt_.set_psd_one_callback([this](int i, uint16_t p) { on_psd_one(i, p); });
  rt_.set_program_callback([this](bool start) { on_program(start); });
  rt_.set_error_callback([this](int target, uint8_t code, const char *name) {
    on_stm32_error(target, code, name);
  });
  rt_.set_log_callback([this](const std::string &m) {
    RCLCPP_INFO(get_logger(), "%s", m.c_str());
  });

  // 노드 실행 시 UART 열기. 이후 1초마다 연결 상태 확인 (해제 처리, 재연결)
  open_port();
  retry_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { check_port(); });
}

TbUartNode::~TbUartNode()
{
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
}

bool TbUartNode::open_port()
{
  if (initialized_) return true;

  // 1. 후보 장치 탐색 (channel 지정 시 그 번호만, -1이면 ttyUSB0~9 중 존재하는 장치)
  std::vector<int> candidates;
  if (channel_ >= 0) {
    candidates.push_back(channel_);
  } else {
    for (int i = 0; i < 10; ++i) {
      const std::string dev = "/dev/ttyUSB" + std::to_string(i);
      if (access(dev.c_str(), F_OK) == 0) candidates.push_back(i);
    }
  }

  if (candidates.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "No /dev/ttyUSB* device found. Connect the USB-UART adapter.");
    return false;
  }

  // 2. 권한 검사 후 열기
  bool permission_denied = false;
  for (int ch : candidates) {
    const std::string dev = "/dev/ttyUSB" + std::to_string(ch);

    if (access(dev.c_str(), R_OK | W_OK) != 0) {
      if (errno == EACCES) permission_denied = true;
      continue;
    }

    if (rt_.rt_u2s_init(ch, BAUDRATE)) {
      disconnected_ = false;
      initialized_ = true;
      running_ = true;
      rx_thread_ = std::thread(&TbUartNode::rx_loop, this);
      RCLCPP_INFO(get_logger(), "UART opened: %s", dev.c_str());
      return true;
    }
  }

  if (permission_denied) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "UART found but permission denied. Add user to 'dialout' group "
      "(sudo usermod -aG dialout $USER, then re-login) or install the udev rule.");
  }
  return false;
}

void TbUartNode::check_port()
{
  if (!initialized_) {
    open_port();
    return;
  }
  if (disconnected_) close_port();
}

void TbUartNode::close_port()
{
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  rt_.rt_u2s_close();
  initialized_ = false;
  running_state_ = false;   // 작동 상태 초기화
  disconnected_ = false;
  publish_error("UART disconnected");
}

void TbUartNode::publish_text(const std::string &text)
{
  std_msgs::msg::String m;
  m.data = text;
  pub_->publish(m);
  RCLCPP_INFO(get_logger(), "[TOPIC TX] '%s'", text.c_str());
}

void TbUartNode::publish_error(const std::string &msg)
{
  RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
  publish_text("error " + msg);
}

void TbUartNode::on_command(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string &s = msg->data;
  RCLCPP_INFO(get_logger(), "[TOPIC RX] '%s'", s.c_str());

  const std::vector<std::string> t = split(s);
  if (t.empty()) {
    publish_error("empty command");
    return;
  }

  const size_t n = t.size();
  int a = 0, b = 0, c = 0;

  // ---------- start ----------
  if (n == 1 && t[0] == "start") {
    cmd_start();
    return;
  }

  // ---------- start 외 명령은 작동 중에만 수행 ----------
  if (!running_state_) {
    // 로그는 출력하되 rt_u2s_uart로는 송신하지 않음
    publish_error("STM32 is not running. Command ignored: '" + s + "' (send 'start' first)");
    return;
  }

  // ---------- quit ----------
  if (n == 1 && t[0] == "quit") {
    cmd_quit();
    return;
  }

  // ---------- filter <N> ----------
  if (t[0] == "filter" && n == 2 && to_int(t[1], a)) {
    if (a < 1 || a > 65535) {
      publish_error("filter size out of range (1 ~ 65535): " + std::to_string(a));
      return;
    }
    rt_.send_filter_size(static_cast<uint16_t>(a));
    return;
  }

  // ---------- direction ... ----------
  if (t[0] == "direction") {
    // direction <L> <R>
    if (n == 3 && to_int(t[1], a) && to_int(t[2], b)) {
      if ((a != 1 && a != -1) || (b != 1 && b != -1)) {
        publish_error("invalid direction (1 or -1): " + std::to_string(a) + " " + std::to_string(b));
        return;
      }
      std::lock_guard<std::mutex> lk(mtx_);
      dirL_ = a;
      dirR_ = b;
      RCLCPP_INFO(get_logger(), "direction set: dirL=%d dirR=%d", dirL_, dirR_);
      return;
    }

    // direction L <v> / direction R <v>
    if (n == 3 && (t[1] == "L" || t[1] == "R") && to_int(t[2], a)) {
      if (a != 1 && a != -1) {
        publish_error("invalid direction (1 or -1): " + std::to_string(a));
        return;
      }
      std::lock_guard<std::mutex> lk(mtx_);
      if (t[1] == "L") dirL_ = a;
      else dirR_ = a;
      RCLCPP_INFO(get_logger(), "direction set: dirL=%d dirR=%d", dirL_, dirR_);
      return;
    }
  }

  // ---------- velocity ... ----------
  if (t[0] == "velocity") {
    // velocity period <ms>
    if (n == 3 && t[1] == "period" && to_int(t[2], a)) {
      if (a < 0 || a > 65535) { publish_error("period out of range: " + std::to_string(a)); return; }
      rt_.send_velocity_period(static_cast<uint16_t>(a));
      return;
    }

    // velocity id set <idL> <idR>
    if (n == 5 && t[1] == "id" && t[2] == "set" && to_int(t[3], a) && to_int(t[4], b)) {
      if (a < 0 || a > 1 || b < 0 || b > 1 || a == b) {
        publish_error("invalid velocity id: " + std::to_string(a) + " " + std::to_string(b));
        return;
      }
      std::lock_guard<std::mutex> lk(mtx_);
      idL_ = a;
      idR_ = b;
      RCLCPP_INFO(get_logger(), "velocity id set: idL=%d idR=%d", idL_, idR_);
      return;
    }

    // velocity L <v> / velocity R <v>
    if (n == 3 && (t[1] == "L" || t[1] == "R") && to_int(t[2], a)) {
      int id, v;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (t[1] == "L") { id = idL_; v = a * dirL_; }
        else             { id = idR_; v = a * dirR_; }
      }
      rt_.send_velocity(id, v);
      return;
    }

    // velocity <L> <R>
    if (n == 3 && to_int(t[1], a) && to_int(t[2], b)) {
      int vel[2];
      {
        std::lock_guard<std::mutex> lk(mtx_);
        vel[idL_] = a * dirL_;   // Lvelocity -> idL 위치 (방향 곱)
        vel[idR_] = b * dirR_;   // Rvelocity -> idR 위치 (방향 곱)
      }
      rt_.send_velocities(vel);
      return;
    }
  }

  // ---------- psd ... ----------
  if (t[0] == "psd") {
    // psd period <ms>
    if (n == 3 && t[1] == "period" && to_int(t[2], a)) {
      if (a < 0 || a > 65535) { publish_error("period out of range: " + std::to_string(a)); return; }
      rt_.send_psd_period(static_cast<uint16_t>(a));
      return;
    }

    // psd id set <idF> <idL> <idR>
    if (n == 6 && t[1] == "id" && t[2] == "set" &&
        to_int(t[3], a) && to_int(t[4], b) && to_int(t[5], c)) {
      if (a < 0 || a > 2 || b < 0 || b > 2 || c < 0 || c > 2 ||
          a == b || b == c || a == c) {
        publish_error("invalid psd id: " + std::to_string(a) + " " +
                      std::to_string(b) + " " + std::to_string(c));
        return;
      }
      std::lock_guard<std::mutex> lk(mtx_);
      psdF_ = a;
      psdL_ = b;
      psdR_ = c;
      RCLCPP_INFO(get_logger(), "psd id set: psdF=%d psdL=%d psdR=%d", psdF_, psdL_, psdR_);
      return;
    }
  }

  // ---------- get ... ----------
  if (t[0] == "get" && n >= 2) {
    if (t[1] == "velocity") {
      if (n == 2) {
        rt_.request_velocities();
        return;
      }
      if (n == 3 && (t[2] == "L" || t[2] == "R")) {
        int id;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          id = (t[2] == "L") ? idL_ : idR_;
        }
        rt_.request_velocity(id);
        return;
      }
    }

    if (t[1] == "psd") {
      if (n == 2) {
        rt_.request_psds();
        return;
      }
      if (n == 3 && (t[2] == "F" || t[2] == "L" || t[2] == "R")) {
        int id;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          id = (t[2] == "F") ? psdF_ : (t[2] == "L") ? psdL_ : psdR_;
        }
        rt_.request_psd(id);
        return;
      }
    }
  }

  publish_error("unknown command: '" + s + "'");
}

void TbUartNode::cmd_start()
{
  if (!open_port()) {
    publish_error("UART not opened");
    return;
  }
  rt_.send_start();
}

void TbUartNode::cmd_quit()
{
  rt_.send_end();
}

void TbUartNode::rx_loop()
{
  while (running_ && !disconnected_ && rclcpp::ok()) {
    if (rt_.receive(100) < 0) {
      disconnected_ = true;   // 오류 반복 출력 대신 한 번만 감지, 정리는 check_port()에서
    }
  }
}

void TbUartNode::on_velocity(const int16_t v[2])
{
  std::lock_guard<std::mutex> lk(mtx_);
  publish_text("velocity " + std::to_string(v[idL_]) + " " + std::to_string(v[idR_]));
}

void TbUartNode::on_psd(const uint16_t p[3])
{
  std::lock_guard<std::mutex> lk(mtx_);
  publish_text("psd " + std::to_string(p[psdF_]) + " " +
               std::to_string(p[psdL_]) + " " + std::to_string(p[psdR_]));
}

void TbUartNode::on_velocity_one(int index, int16_t value)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (index == idL_) {
    publish_text("velocity L " + std::to_string(value));
  } else if (index == idR_) {
    publish_text("velocity R " + std::to_string(value));
  }
}

void TbUartNode::on_psd_one(int index, uint16_t value)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (index == psdF_) {
    publish_text("psd F " + std::to_string(value));
  } else if (index == psdL_) {
    publish_text("psd L " + std::to_string(value));
  } else if (index == psdR_) {
    publish_text("psd R " + std::to_string(value));
  }
}

void TbUartNode::on_program(bool start)
{
  running_state_ = start;
  RCLCPP_INFO(get_logger(), "STM32 state: %s", start ? "running" : "stopped");
  publish_text(start ? "stm32 start" : "stm32 quit");
}

void TbUartNode::on_stm32_error(int target, uint8_t code, const char *name)
{
  static const char *const targets[] = {"common", "velocity", "psd"};
  const std::string msg = std::string("stm32 ") + targets[target] + " " + name;
  RCLCPP_ERROR(get_logger(), "STM32 error: %s (code 0x%02X)", msg.c_str(), code);
  publish_text("error " + msg);
}

}  // namespace tb