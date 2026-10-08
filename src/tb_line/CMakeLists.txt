cmake_minimum_required(VERSION 3.8)
project(tb_line)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tb_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lineDetect_node src/lineDetect_node.cpp)
target_include_directories(lineDetect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(lineDetect_node rclcpp sensor_msgs cv_bridge tb_interfaces)
target_link_libraries(lineDetect_node ${OpenCV_LIBS})

install(TARGETS lineDetect_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch scripts DESTINATION share/${PROJECT_NAME})

ament_package()
