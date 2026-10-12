cmake_minimum_required(VERSION 3.8)
project(tb_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tb_interfaces REQUIRED)

add_executable(brain_node src/brain_node.cpp)
target_compile_features(brain_node PUBLIC cxx_std_17)
ament_target_dependencies(brain_node rclcpp std_msgs tb_interfaces)

install(TARGETS brain_node DESTINATION lib/${PROJECT_NAME})

ament_package()