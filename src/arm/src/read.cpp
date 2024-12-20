#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
#include "serial_setup.h"

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
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/Pose1", 5);
    
    serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error %i from open: %s", errno, strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Port opened");
    }

    serial_setup(serial_port);

    const uint8_t MSG_LEN = 48;
    uint8_t temp_read[MSG_LEN];
    uint8_t zeros = 0;
    uint32_t reset_wait_count = 0;

    while (zeros < MSG_LEN) {
        if (reset_wait_count % MSG_LEN == 0) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for reset...");
        }
        reset_wait_count++;
        
        read(serial_port, temp_read, 1);
        if (temp_read[0] == 0) {
            zeros++;
        } else {
            zeros = 0;
        }
    }
    zeros = 0;

    uint8_t read_buf[MSG_LEN];
    memset(&read_buf, '\0', sizeof(read_buf));
    Pose3 pose{};

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "starting");
    while (rclcpp::ok()) {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "bigloop");
        // I'm 99% sure there's a bug in this logic but it works for now lol
        uint8_t total_bytes_read = 0;
        while (total_bytes_read < MSG_LEN) {
            // read as much as possible into temp_read
            int n = read(serial_port, temp_read, MSG_LEN - total_bytes_read);
            if (n < 0) {
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), strerror(errno));
                return 1;
            }

            uint8_t copy_start = 0;
            
            for (uint8_t i = 0; i < n; i++) {
                if (zeros == MSG_LEN) {
                    zeros = 0;
                    copy_start = i;
                    break;
                }
                if (read_buf[i] == 0) {
                    zeros++;
                } else {
                    zeros = 0;
                }
            }

            memcpy(read_buf + total_bytes_read, temp_read + copy_start, n - copy_start);
            total_bytes_read += n - copy_start;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "loop %d %d %d %d", total_bytes_read, n, zeros, copy_start);
        }

        if (read_buf[0] == 0){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reset");
            continue;
        }

        std::array<float, MSG_LEN / 4 - 1> values{};
        std::memcpy(values.data(), read_buf + 2, sizeof(read_buf));
        // int32_t us;
        // std::memcpy(&us, read_buf + 4, sizeof(int32_t));

        values[0] /= 100;
        values[1] /= 100;
        values[2] /= 100;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = values[0];
        pose.pose.position.y = values[1];
        pose.pose.position.z = values[2];
        pose.pose.orientation.x = values[3];
        pose.pose.orientation.y = values[4];
        pose.pose.orientation.z = values[5];
        pose.pose.orientation.w = values[6];
        pose.header.stamp = node->now();

        publisher->publish(pose);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "us: %d Published [%f %f %f %f]", (int)us, values[4], values[5], values[6]);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%f %f %f] [%f %f %f %f] [%f %f %f]", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9]);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%f %f %f] [%f %f %f] [%f %f %f] [%f %f]", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10]);
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    close(serial_port);
    return 0;
}
