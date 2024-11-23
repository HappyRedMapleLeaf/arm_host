#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp"
#include "kinematics.h"

#include <chrono>
using namespace std::chrono_literals;

#include <ncurses.h>


void sigterm_handler(int signum);
int main(int argc, char **argv);


int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("key_teleop");
    rclcpp::Client<arm_interfaces::srv::ServoUpdate>::SharedPtr client =
    node->create_client<arm_interfaces::srv::ServoUpdate>("servo_set_srv");

    // Initialize ncurses
    initscr();
    cbreak();
    noecho();
    timeout(0); // Non-blocking input
    keypad(stdscr, TRUE);

    Pose3 current_pose{Vec3(140, 60, 40), RotationMatrix(Vec3(-M_PI_2, -M_PI, 0))};
    Angles angles{};
    double claw_angle = 0.0;
    uint64_t last_updated = node->now().nanoseconds() / 1000000;

    const double linear_speed = 2.0;                  // mm
    const double rotate_speed = M_PI/180.0 * 2;       // rad
    const double claw_speed = M_PI/180.0 * 5;         // rad
    const Limits limits = {
        {{0.0,       3*M_PI/2},
        {0,         M_PI},
        {-M_PI_4,   5*M_PI/4},
        {-3*M_PI_4, 3*M_PI_4},
        {-M_PI_2,   M_PI_2},
        {-M_PI_2,   M_PI_2}}
    };
    
    while (rclcpp::ok()) {
        uint64_t milliseconds = node->now().nanoseconds() / 1000000;

        if (milliseconds - last_updated > 20) {
            int32_t key = getch();

            Pose3 oldpose = current_pose;

            bool changed = true;

            switch (key) {
                case 'q':
                    current_pose.pos.x += linear_speed;
                    break;
                case 'a':
                    current_pose.pos.x -= linear_speed;
                    break;
                case 'w':
                    current_pose.pos.y += linear_speed;
                    break;
                case 's':
                    current_pose.pos.y -= linear_speed;
                    break;
                case 'e':
                    current_pose.pos.z += linear_speed;
                    break;
                case 'd':
                    current_pose.pos.z -= linear_speed;
                    break;
                case 'r':
                    current_pose.dir = RotationMatrix(Vec3( rotate_speed, 0, 0)).mul(current_pose.dir);
                    break;
                case 'f':
                    current_pose.dir = RotationMatrix(Vec3(-rotate_speed, 0, 0)).mul(current_pose.dir);
                    break;
                case 't':
                    current_pose.dir = RotationMatrix(Vec3(0,  rotate_speed, 0)).mul(current_pose.dir);
                    break;
                case 'g':
                    current_pose.dir = RotationMatrix(Vec3(0, -rotate_speed, 0)).mul(current_pose.dir);
                    break;
                case 'y':
                    current_pose.dir = RotationMatrix(Vec3(0, 0,  rotate_speed)).mul(current_pose.dir);
                    break;
                case 'h':
                    current_pose.dir = RotationMatrix(Vec3(0, 0, -rotate_speed)).mul(current_pose.dir);
                    break;
                case 'u':
                    claw_angle += claw_speed;
                    break;
                case 'j':
                    claw_angle -= claw_speed;
                    break;
                default:
                    changed = false;
                    break;
            }

            if (changed) {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New xyz: [%f, %f, %f]", 
                //     current_pose.pos.x, current_pose.pos.y, current_pose.pos.z
                // );
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "c");
            }

            claw_angle = std::clamp(claw_angle, -M_PI_2, M_PI_2);

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

            // Wait for result
            if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service write_server");
            }
        }
    }

    endwin();
    rclcpp::shutdown();
    return 0;
}

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    endwin();
    rclcpp::shutdown();
    exit(signum);
}