cmake_minimum_required(VERSION 3.16)
project(tb_signs LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tb_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)
find_package(yaml-cpp REQUIRED)

# ---- CUDA / TensorRT: Jetson(Nano)에만 있음. 없으면 노드 빌드를 건너뜀
if(EXISTS /usr/local/cuda)
  set(CUDAToolkit_ROOT /usr/local/cuda)
endif()
find_package(CUDAToolkit QUIET)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include /usr/local/tensorrt/include)
find_library(NVINFER_LIB nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/local/tensorrt/lib)

if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND NVINFER_LIB)
  message(STATUS "TensorRT: ${TENSORRT_INCLUDE_DIR} / ${NVINFER_LIB}")

  add_executable(sign_detect_node
    src/sign_detect_node.cpp
    src/trt_detector.cpp)
  target_include_directories(sign_detect_node PRIVATE include ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(sign_detect_node ${OpenCV_LIBS} yaml-cpp ${NVINFER_LIB} CUDA::cudart)
  ament_target_dependencies(sign_detect_node
    rclcpp sensor_msgs tb_interfaces cv_bridge ament_index_cpp)
  install(TARGETS sign_detect_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "CUDA/TensorRT 없음: sign_detect_node 빌드 건너뜀 (Jetson에서만 빌드됨)")
endif()

install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

ament_package()