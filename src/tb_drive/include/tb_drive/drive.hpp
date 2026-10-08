#ifndef TB_DRIVE__DRIVE_HPP_
#define TB_DRIVE__DRIVE_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TbDrive : public rclcpp::Node
{
public:
  TbDrive();
  ~TbDrive();

private:
  void timer_callback();
  void on_status(const std_msgs::msg::String::SharedPtr msg);
  void send(const std::string & cmd);
  void set_velocity(int left, int right);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool running_ = false;      // STM32가 "stm32 start" 응답했는지
  bool configured_ = false;   // psd 주기 설정 완료 여부
  int psd_f_ = 0, psd_l_ = 0, psd_r_ = 0;
  bool psd_valid_ = false;


};

#endif