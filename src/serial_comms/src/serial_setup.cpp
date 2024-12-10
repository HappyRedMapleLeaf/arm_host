#include "rclcpp/rclcpp.hpp"

#include <errno.h>
#include <termios.h>
#include <unistd.h>

void serial_setup(int serial_port) {
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if (tcgetattr(serial_port, &tty) != 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error %i from tcgetattr: %s", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    
    tty.c_cc[VTIME] = 10; // 1s timeout
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 230400
    // cfsetispeed(&tty, B230400);
    // cfsetospeed(&tty, B230400);
    cfsetspeed(&tty, B230400); // apparently only exists on some Linux systems. We ball
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "flags set");

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error %i from tcsetattr: %s", errno, strerror(errno));
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "flags saved");
}
