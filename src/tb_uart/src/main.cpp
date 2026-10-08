#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tb_uart/tb_uart_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb::TbUartNode>());
  rclcpp::shutdown();
  return 0;
}