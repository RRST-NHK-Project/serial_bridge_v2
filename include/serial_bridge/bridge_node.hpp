#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <chrono>
#include <deque>
#include <string>

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode(uint8_t device_id, const std::string &port);

private:
    void update();
    void tx_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    bool try_open_port();
    void close_port();

    int fd_;
    uint8_t device_id_;
    std::string port_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<uint8_t> rx_buffer_;

    // pub,sub
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tx_sub_;
};
