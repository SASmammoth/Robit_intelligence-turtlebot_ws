cmake_minimum_required(VERSION 3.8)
project(tb_uart)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tb_uart_node
  src/main.cpp
  src/tb_uart_node.cpp
  src/rt_u2s_uart.cpp
  src/ubuntu_uart.cpp)

target_include_directories(tb_uart_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(tb_uart_node PUBLIC cxx_std_17)
ament_target_dependencies(tb_uart_node rclcpp std_msgs)

install(TARGETS tb_uart_node DESTINATION lib/${PROJECT_NAME})

ament_package()