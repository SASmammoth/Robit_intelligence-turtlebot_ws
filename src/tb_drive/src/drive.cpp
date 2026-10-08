#include "tb_drive/drive.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

TbDrive::TbDrive() : Node("tb_drive")
{
  this->declare_parameter<int>("forward_speed", 40);
  this->declare_parameter<int>("turn_speed", 40);
  pub_ = this->create_publisher<std_msgs::msg::String>("TB_Uart_RX", 10);
  sub_ = this->create_subscription<std_msgs::msg::String>("TB_Uart_TX", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_status(msg); });
  timer_ = this->create_wall_timer(100ms, std::bind(&TbDrive::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "tb_drive node started");
}

TbDrive::~TbDrive()
{
  if (running_) {
    set_velocity(0, 0);
    send("quit");
  }
}

void TbDrive::send(const std::string & cmd)
{
  std_msgs::msg::String m;
  m.data = cmd;
  pub_->publish(m);
}

void TbDrive::set_velocity(int left, int right)
{
  // 모터 장착 방향 보정: 반대로 도는 쪽을 -1로
  const int L_DIR = 1;
  const int R_DIR = -1;

  send("velocity L " + std::to_string(left  * L_DIR));
  send("velocity R " + std::to_string(right * R_DIR));
}

void TbDrive::on_status(const std_msgs::msg::String::SharedPtr msg)
{
  std::istringstream iss(msg->data);
  std::string head;
  iss >> head;

  if (msg->data == "stm32 start") {
    running_ = true;
    RCLCPP_INFO(this->get_logger(), "STM32 running");
  } else if (msg->data == "stm32 quit") {
    running_ = false;
    configured_ = false;
  } else if (head == "psd") {
    int f, l, r;
    if (iss >> f >> l >> r) {          // "psd F L R" 형식만 (psd F <v> 같은 개별값은 무시)
      psd_f_ = f; psd_l_ = l; psd_r_ = r;
      psd_valid_ = true;
    }
  } else if (head == "error") {
    RCLCPP_WARN(this->get_logger(), "%s", msg->data.c_str());
  }
}

void TbDrive::timer_callback()
{
  // 1단계: STM32 시작될 때까지 1초마다 start 전송
  if (!running_) {
    static int tick = 0;
    if (tick++ % 10 == 0) send("start");
    return;
  }

  // 2단계: 시작 직후 한 번 PSD 자동 전송 주기 설정
  if (!configured_) {
    send("psd period 100");
    configured_ = true;
    return;
  }

  // 3단계: 주행 로직
  if (!psd_valid_) return;

  // 매 주기마다 현재 파라미터 값을 읽음
  const int forward = this->get_parameter("forward_speed").as_int();
  const int turn    = this->get_parameter("turn_speed").as_int();

  const int FRONT_LIMIT = 1500;
  if (psd_f_ > FRONT_LIMIT) {
    if (psd_l_ < psd_r_) set_velocity(-turn, turn);
    else                 set_velocity(turn, -turn);
  } else {
    set_velocity(forward, forward);
  }
}