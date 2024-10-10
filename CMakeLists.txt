cmake_minimum_required(VERSION 3.8)
project(ultrasonic_reader)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(ultrasonic_reader_node src/main.cpp)

ament_target_dependencies(ultrasonic_reader_node
  rclcpp
  sensor_msgs
)

target_link_libraries(ultrasonic_reader_node
  ${Boost_LIBRARIES}
)

install(TARGETS
  ultrasonic_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
