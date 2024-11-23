#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp"

#include "kinematics.h"

#include <cstdlib>
#include <memory>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <termios.h>

#include <csignal>

#include <map>

#include <chrono>
using namespace std::chrono_literals;

Pose3 current_pose{Vec3(140, 60, 40), RotationMatrix(Vec3(-M_PI_2, -M_PI, 0))};
double claw_angle = 0.0;
Angles angles{};
double direction = 1.0;

const Limits limits = {
    {{0.0,       3*M_PI/2},
     {0,         M_PI},
     {-M_PI_4,   5*M_PI/4},
     {-3*M_PI_4, 3*M_PI_4},
     {-M_PI_2,   M_PI_2},
     {-M_PI_2,   M_PI_2}}
};

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv);

int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("key_teleop");
    rclcpp::Client<arm_interfaces::srv::ServoUpdate>::SharedPtr client =
    node->create_client<arm_interfaces::srv::ServoUpdate>("servo_set_srv");

    uint64_t last_updated = node->now().nanoseconds() / 1000000;
    
    while (rclcpp::ok()) {
        uint64_t milliseconds = node->now().nanoseconds() / 1000000;

        if (milliseconds - last_updated > 10) {
            Pose3 oldpose = current_pose;

            last_updated = milliseconds;
            if (current_pose.pos.z > 200) {
                direction = -1.0;
            } else if (current_pose.pos.z < 40) {
                direction = +1.0;
            }
            
            current_pose.pos.z += direction;

            bool success = InvKin(current_pose, limits, angles);

            if (!success) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Destination out of range");
                current_pose = oldpose;
                continue;
            }

            auto request = std::make_shared<arm_interfaces::srv::ServoUpdate::Request>();

            for (int i = 0; i < 6; i++) {
                request->servo_angles[i] = angles[i];
            }
            request->servo_angles[6] = claw_angle;

            while (!client->wait_for_service(20ms)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return 0;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result_data = result.get();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service write_server");
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}