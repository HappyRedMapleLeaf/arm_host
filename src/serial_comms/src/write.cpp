#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/pos_update.hpp" // so why ON EARTH IS IT pos_update instead of posupdate or PosUpdate??? WHO ASKED IT TO ADD AN UNDERSCORE

#include <cstdlib>
#include <memory>

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>

int serial_port;

double x = 29.9;
double y = -147;
double z = 38.2;

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    close(serial_port);
    rclcpp::shutdown();
    exit(signum);
}

void change_pos(const std::shared_ptr<arm_interfaces::srv::PosUpdate::Request>  request,
                      std::shared_ptr<arm_interfaces::srv::PosUpdate::Response> response)
{
    x += request->dx;
    y += request->dy;
    z += request->dz;

    double data[4];
    data[0] = x;
    data[1] = y;
    data[2] = z;
    data[3] = 0.0;  // unused byte

    write(serial_port, data, 32);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "data written");

    response->new_x = x;
    response->new_y = y;
    response->new_z = z;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request: dx=%f, dy=%f, dz=%f", request->dx, request->dy, request->dz);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response: [%f, %f, %f]", response->new_x, response->new_y, response->new_z);
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

    rclcpp::Service<arm_interfaces::srv::PosUpdate>::SharedPtr service =
        node->create_service<arm_interfaces::srv::PosUpdate>("key_teleop_srv", &change_pos);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready...");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
