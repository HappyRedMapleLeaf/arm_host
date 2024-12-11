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
Pose3 current_pose{Vec3(-170, 28, 34), RotationMatrix(Vec3(0, M_PI, M_PI_2))};
double claw_angle = 0.0;
Angles angles{};

uint32_t period = 6000;
uint32_t t = 0;
float r = 60;
float z_offset = 0;
float y_offset = -20;
            
double write_data[8];

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

    // send command 1 to set servo angles
    uint64_t command = 0x01UL;
    std::memcpy(&write_data[0], &command, sizeof(command));

    bool success = InvKin(current_pose, limits, angles);
    for (int i = 0; i < 6; i++) {
        write_data[i+1] = angles[i];
    }
    write_data[7] = claw_angle;
    write(serial_port, write_data, 64);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pen_circle_demo");

    uint64_t last_updated = node->now().nanoseconds() / 1000000;

    // give time to hand the pen to the robot
    while (node->now().nanoseconds() / 1000000 - last_updated < 2000) {}

    claw_angle = -0.5;
    for (int i = 0; i < 6; i++) {
        write_data[i+1] = angles[i];
    }
    write_data[7] = claw_angle;
    write(serial_port, write_data, 64);

    while (node->now().nanoseconds() / 1000000 - last_updated < 4000) {}
    last_updated = node->now().nanoseconds() / 1000000;
    
    while (rclcpp::ok()) {
        uint64_t milliseconds = node->now().nanoseconds() / 1000000;

        if (milliseconds - last_updated > 20) {            
            Pose3 oldpose = current_pose;
            
            t += milliseconds - last_updated;
            current_pose.pos.z = r * sin(t * 2 * M_PI / period) + z_offset;
            current_pose.pos.y = r * cos(t * 2 * M_PI / period) + y_offset;
            last_updated = milliseconds;

            bool success = InvKin(current_pose, limits, angles);

            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Destination out of range");
                current_pose = oldpose;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f %f %f", current_pose.pos.x, current_pose.pos.y, current_pose.pos.z);
            }

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