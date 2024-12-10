cmake_minimum_required(VERSION 3.8)
project(serial_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LIB_SOURCES src/kinematics.cpp src/math_utils.cpp src/serial_setup.cpp)
set(LIB_HEADERS src/kinematics.h src/math_utils.h src/serial_setup.h)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(Curses REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(rviz_test src/rviz_test.cpp)
ament_target_dependencies(rviz_test rclcpp geometry_msgs)

add_executable(read src/read.cpp ${LIB_SOURCES})
ament_target_dependencies(read rclcpp geometry_msgs)

add_executable(read_debug src/read_debug.cpp ${LIB_SOURCES})
ament_target_dependencies(read_debug rclcpp geometry_msgs)

add_executable(write src/write.cpp ${LIB_SOURCES})
ament_target_dependencies(write rclcpp arm_interfaces)

add_executable(key_teleop src/key_teleop.cpp ${LIB_SOURCES})
ament_target_dependencies(key_teleop rclcpp arm_interfaces)
target_link_libraries(key_teleop ${CURSES_LIBRARIES})

add_executable(linear_motion_test src/linear_motion_test.cpp  ${LIB_SOURCES})
ament_target_dependencies(linear_motion_test rclcpp arm_interfaces)

add_executable(servo_set src/servo_set.cpp)
ament_target_dependencies(servo_set rclcpp arm_interfaces)

install(TARGETS
    read
    read_debug
    write
    key_teleop
    linear_motion_test
    servo_set
    rviz_test
    DESTINATION lib/${PROJECT_NAME})

ament_package()
