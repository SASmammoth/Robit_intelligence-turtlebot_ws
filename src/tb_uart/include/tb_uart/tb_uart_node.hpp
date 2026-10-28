#ifndef TB_UART__TB_UART_NODE_HPP_
#define TB_UART__TB_UART_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "tb_uart/rt_u2s_uart.hpp"
#include "tb_uart/wheel_odom.hpp"

namespace tb {

class TbUartNode : public rclcpp::Node {
public:
  TbUartNode();
  ~TbUartNode() override;

private:
  void on_command(const std_msgs::msg::String::SharedPtr msg);
  bool open_port();   // 포트 열기 (channel_ == -1이면 ttyUSB0~9 자동 탐색)
  void check_port();                          // 1초마다 호출: 연결 해제 처리, 재연결
  void close_port();
  void publish_error(const std::string &msg); // 에러 로그 + 토픽 발행
  void cmd_start();
  void cmd_quit();

  void rx_loop();
  void on_velocity(const int16_t v[2]);
  void on_psd(const uint16_t p[3]);
  void on_program(bool start);
  void publish_text(const std::string &text);
  void on_velocity_one(int index, int16_t value);
  void on_psd_one(int index, uint16_t value);
  void on_stm32_error(int target, uint8_t code, const char *name);
  void publish_odom(const rclcpp::Time &t);
  void odom_watch();   // 200ms마다: velocity 피드백이 안 오면 주기 설정을 다시 보냄
  uint16_t effective_vel_period(int requested) const;

  int channel_{0};
  static constexpr unsigned int BAUDRATE = 1000000;

  RtU2sUart rt_;
  bool initialized_{false};

  std::atomic<bool> running_state_{false};   // STM32 작동 상태 (시작 수신: true, 종료 수신: false)
  std::atomic<bool> disconnected_{false};    // rx 스레드가 연결 해제를 감지하면 true

  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  // ID 설정 (rx 스레드와 공유)
  std::mutex mtx_;
  int idL_{0}, idR_{1};
  int psdF_{0}, psdL_{1}, psdR_{2};
  int dirL_{1}, dirR_{-1};   // velocity 송신 방향 (1 또는 -1)

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;   // 1초마다 연결 상태 확인

  // ====== odom (velocity 피드백 적분 → /odom)
  bool log_data_{false};       // false면 velocity·psd 데이터 로그를 안 찍음
  bool odom_enable_{true};
  int odom_period_ms_{10};     // STM32에 요청하는 velocity 피드백 주기
  int text_period_ms_{33};     // TB_Uart_TX "velocity L R" 문자열 발행 간격 (brain 주차용, GUI 부하 방지)
  int user_vel_period_{0};     // TB_Uart_RX로 받은 마지막 "velocity period" 값 (0 = 요청 없음)
  WheelOdom odom_;             // rx 스레드에서만 갱신 (다른 스레드는 last_fb_ns_만 읽음)
  std::atomic<int64_t> last_fb_ns_{0};   // 마지막 velocity 피드백 받은 시각 (steady)
  int64_t last_period_sent_ns_{0};
  rclcpp::Time last_text_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_reset_sub_;
  std::atomic<bool> odom_reset_req_{false};
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

}  // namespace tb

#endif  // TB_UART__TB_UART_NODE_HPP_