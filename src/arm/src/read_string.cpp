#include "rclcpp/rclcpp.hpp"
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

int serial_port;

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    close(serial_port);
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("read");
    
    serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error %i from open: %s", errno, strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Port opened");
    }

    serial_setup(serial_port);
    
    for (;;) {
        char read_buf [128];

        memset(&read_buf, '\0', sizeof(read_buf));

        int n = read(serial_port, &read_buf, sizeof(read_buf));

        if (n < 0) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), strerror(errno));
            return 1;
        } else if (n > 0) {
            printf("%s", read_buf);
        }
    }

    rclcpp::shutdown();
    return 0;
}