#include "tb_drive/drive.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TbDrive>());
  rclcpp::shutdown();
  return 0;
}