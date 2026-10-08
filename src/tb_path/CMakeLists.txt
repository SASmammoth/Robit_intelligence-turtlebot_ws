cmake_minimum_required(VERSION 3.8)
project(tb_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # 스켈레톤 연산이 많아서 최적화 필수
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tb_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# ROS와 무관한 알고리즘 (노트북에서 단독 테스트 가능)
add_library(path_core STATIC
  src/laneSkeleton.cpp
  src/lanePath.cpp
  src/obstacles.cpp)
target_include_directories(path_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(path_core ${OpenCV_LIBS})

add_executable(path_node src/path_node.cpp)
ament_target_dependencies(path_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs cv_bridge tb_interfaces)
target_link_libraries(path_node path_core)

install(TARGETS path_node DESTINATION lib/${PROJECT_NAME})

ament_package()
