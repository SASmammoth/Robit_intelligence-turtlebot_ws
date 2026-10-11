cmake_minimum_required(VERSION 3.8)
project(tb_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tb_interfaces REQUIRED)

add_executable(drive_node
  src/drive_node.cpp
  src/pure_pursuit.cpp
)
target_include_directories(drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(drive_node PUBLIC cxx_std_17)
ament_target_dependencies(drive_node
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs tb_interfaces
)

install(TARGETS drive_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
