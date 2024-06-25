#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/pos_update.hpp" // so why ON EARTH IS IT pos_update instead of posupdate or PosUpdate??? WHO ASKED IT TO ADD AN UNDERSCORE

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

std::map<char, std::vector<float>> moveBindings
{
  {'q', {5.0, 0.0, 0.0}},
  {'a', {-5.0, 0.0, 0.0}},
  {'w', {0.0, 5.0, 0.0}},
  {'s', {0.0, -5.0, 0.0}},
  {'e', {0.0, 0.0, 5.0}},
  {'d', {0.0, 0.0, -5.0}}
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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("key_teleop");
    rclcpp::Client<arm_interfaces::srv::PosUpdate>::SharedPtr client =
    node->create_client<arm_interfaces::srv::PosUpdate>("key_teleop_srv");
    
    while (rclcpp::ok()) {
        int key = getch();

        if (key != EOF && moveBindings.count(key) == 1) {
            auto request = std::make_shared<arm_interfaces::srv::PosUpdate::Request>();
            request->dx = moveBindings[key][0];
            request->dy = moveBindings[key][1];
            request->dz = moveBindings[key][2];

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request");

            while (!client->wait_for_service(1s)) {
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
                auto result_print = result.get();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New position: [%f, %f, %f]", result_print->new_x, result_print->new_y, result_print->new_z);
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