#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp"
#include "serial_setup.h"

#include <cstdlib>
#include <memory>

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>

#include <chrono>

using namespace std::chrono_literals;

int serial_port;

double write_data[8];
bool new_data = false;

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    close(serial_port);
    rclcpp::shutdown();
    exit(signum);
}

void change_pos(const std::shared_ptr<arm_interfaces::srv::ServoUpdate::Request>  request,
                      std::shared_ptr<arm_interfaces::srv::ServoUpdate::Response> response) {
    // command: set servo angles
    uint64_t command = 0x01UL;
    std::memcpy(&write_data[0], &command, sizeof(command));

    for (int i = 0; i < 7; i++) {
        write_data[i+1] = request->servo_angles[i];
    }
    
    new_data = true;
    response->success = true;
}

int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);

    rclcpp::init(argc, argv);
    
    serial_port = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error %i from open: %s", errno, strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Port opened");
    }

    serial_setup(serial_port);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("write");

    rclcpp::Service<arm_interfaces::srv::ServoUpdate>::SharedPtr service =
        node->create_service<arm_interfaces::srv::ServoUpdate>("servo_set_srv", &change_pos);

    // uint64_t last_updated = node->now().nanoseconds() / 1000000;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready...");

    // Create a timer to limit the write rate
    auto timer = node->create_wall_timer(20ms, [&]() {
        if (new_data) {
            write(serial_port, write_data, 64);
            new_data = false;
            // uint64_t now = node->now().nanoseconds() / 1000000;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%lu", now - last_updated);
            // last_updated = now;
        }
    });

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
