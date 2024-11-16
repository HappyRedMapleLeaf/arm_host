cmake_minimum_required(VERSION 3.8)
project(serial_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_interfaces REQUIRED)

add_executable(serial_setup src/serial_setup.cpp)
ament_target_dependencies(serial_setup rclcpp)

add_executable(read src/read.cpp)
ament_target_dependencies(read rclcpp)

add_executable(write src/write.cpp)
ament_target_dependencies(write rclcpp arm_interfaces)

add_executable(key_teleop src/key_teleop.cpp)
ament_target_dependencies(key_teleop rclcpp arm_interfaces)

add_executable(servo_set src/servo_set.cpp)
ament_target_dependencies(servo_set rclcpp arm_interfaces)

install(TARGETS
  serial_setup
  read
  write
  key_teleop
  servo_set
  DESTINATION lib/${PROJECT_NAME})

ament_package()
