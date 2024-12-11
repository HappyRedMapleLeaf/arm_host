#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cstdlib>
#include <memory>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <termios.h>

#include <csignal>

#include <map>
#include <algorithm>

#include <chrono>
using namespace std::chrono_literals;

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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rviz_test");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/Pose1", 5);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    rclcpp::Rate rate(1.0); // 1 Hz
    while (rclcpp::ok()) {
        pose.header.stamp = node->now();
        publisher->publish(pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Published pose: [%f, %f, %f]", 
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        rclcpp::spin_some(node);
        pose.pose.position.x += 0.01;
        rate.sleep();
    }
    
    // while (rclcpp::ok()) {
    //     int key = getch();
    // }

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