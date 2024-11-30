#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <memory>

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>

#include "math_utils.h"

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

    uint8_t read_buf[64];
    memset(&read_buf, '\0', sizeof(read_buf));
    Pose3 pose{};
    
    for (;;) {
        int n = read(serial_port, &read_buf, sizeof(read_buf));

        if (n < 0) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), strerror(errno));
            return 1;
        } else if (n > 0) {
            // std::array<double, 8> values{};
            // for (int i = 0; i < 8; i++) {
            //     std::memcpy(&values[i], read_buf + i*8, sizeof(double));
            // }

            // pose.pos.x = values[0];
            // pose.pos.y = values[1];
            // pose.pos.z = values[2];

            // pose.dir.setColumn(2, Vec3(values[3], values[4], values[5]));
            // pose.dir.setColumn(0, Vec3(values[6], values[7], 0));
            // pose.dir[2][0] = pose.dir.getColumn(0).dot(pose.dir.getColumn(2)) / -values[5];
            // pose.dir.setColumn(1, pose.dir.getColumn(2).cross(pose.dir.getColumn(0)));

            // std::cout << pose << std::endl << std::endl;
            printf("%s\n", read_buf);
        }
    }

    rclcpp::shutdown();
    return 0;
}
