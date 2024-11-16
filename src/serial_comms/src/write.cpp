#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp"

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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "data prepared: 0=%f, 1=%f, 2=%f, 3=%f, 4=%f, 5=%f, 6=%f", 
        request->servo_angles[0], request->servo_angles[1], request->servo_angles[2], request->servo_angles[3],
        request->servo_angles[4], request->servo_angles[5], request->servo_angles[6]);
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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("write");

    rclcpp::Service<arm_interfaces::srv::ServoUpdate>::SharedPtr service =
        node->create_service<arm_interfaces::srv::ServoUpdate>("servo_set_srv", &change_pos);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready...");

    // Create a timer to limit the write rate
    auto timer = node->create_wall_timer(50ms, []() {
        if (new_data) {
            write(serial_port, write_data, 64);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "data written");
            new_data = false;
        }
    });

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
