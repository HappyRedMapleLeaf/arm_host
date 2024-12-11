#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp"
#include "serial_setup.h"
#include "kinematics.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <map>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

int serial_port;
Pose3 current_pose{Vec3(140, 60, 40), RotationMatrix(Vec3(-M_PI_2, -M_PI, 0))};
double claw_angle = 0.0;
Angles angles{};
double direction = 3.0;

const Limits limits = {
    {{0.0,       3*M_PI/2},
     {0,         M_PI},
     {-M_PI_4,   5*M_PI/4},
     {-3*M_PI_4, 3*M_PI_4},
     {-M_PI_2,   M_PI_2},
     {-M_PI_2,   M_PI_2}}
};

void sigterm_handler(int signum);
int main(int argc, char **argv);

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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("linear_motion_test");

    uint64_t last_updated = node->now().nanoseconds() / 1000000;
    
    while (rclcpp::ok()) {
        uint64_t milliseconds = node->now().nanoseconds() / 1000000;

        if (milliseconds - last_updated > 20) {
            Pose3 oldpose = current_pose;

            last_updated = milliseconds;
            if (current_pose.pos.z > 200) {
                direction = -3.0;
            } else if (current_pose.pos.z < 40) {
                direction = +3.0;
            }
            
            current_pose.pos.z += direction;

            bool success = InvKin(current_pose, limits, angles);

            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Destination out of range");
                current_pose = oldpose;
                continue;
            }
            
            double write_data[8];

            // send command 1 to set servo angles
            uint64_t command = 0x01UL;
            std::memcpy(&write_data[0], &command, sizeof(command));

            for (int i = 0; i < 6; i++) {
                write_data[i+1] = angles[i];
            }
            write_data[7] = claw_angle;

            write(serial_port, write_data, 64);
        }
    }

    rclcpp::shutdown();
    return 0;
}

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    close(serial_port);
    rclcpp::shutdown();
    exit(signum);
}