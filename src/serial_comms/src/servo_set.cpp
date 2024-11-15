#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/servo_update.hpp" // so why ON EARTH IS IT pos_update instead of posupdate or PosUpdate??? WHO ASKED IT TO ADD AN UNDERSCORE

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

std::array<double, 7> servo_angles = {0, 0, 0, 0, 0, 0, 0};

std::array<std::pair<double, double>, 7> limits = {
    {{0.0, 2*M_PI/3},
     {-M_PI_4, M_PI},
     {-M_PI_4, 5*M_PI/4},
     {-M_PI_2, M_PI_2},
     {-M_PI_2, M_PI_2},
     {-M_PI_2, M_PI_2},
     {0, M_PI}}
};

std::map<char, std::pair<uint8_t, uint8_t>> moveBindings
{
  {'q', {0, true}},
  {'a', {0, false}},
  {'w', {1, true}},
  {'s', {1, false}},
  {'e', {2, true}},
  {'d', {2, false}},
  {'r', {3, true}},
  {'f', {3, false}},
  {'t', {4, true}},
  {'g', {4, false}},
  {'y', {5, true}},
  {'h', {5, false}},
  {'u', {6, true}},
  {'j', {6, false}}
};

void sigterm_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal (%d) received.", signum);
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv);
int getch();

int main(int argc, char **argv) {
    signal(SIGINT, sigterm_handler);

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("servo_set");

    rclcpp::Client<arm_interfaces::srv::ServoUpdate>::SharedPtr client =
        node->create_client<arm_interfaces::srv::ServoUpdate>("servo_set_srv");
    
    while (rclcpp::ok()) {
        int key = getch();

        if (key != EOF && moveBindings.count(key) == 1) {
            auto request = std::make_shared<arm_interfaces::srv::ServoUpdate::Request>();

            request->servo_angles = servo_angles;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request");

            while (!client->wait_for_service(50ms)) {
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
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success: %d New position: [%f, %f, %f, %f, %f, %f, %f]", result_data->success,
                    servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3], servo_angles[4], servo_angles[5], servo_angles[6]
                );
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service write_server");
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON); // disable buffering      
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return ch;
}