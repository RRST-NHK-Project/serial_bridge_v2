#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <string>

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode(uint8_t device_id, const std::string &port,
                     double rx_timeout_sec = 2.0,
                     double reconnect_interval_sec = 3.0);

    // スキャンスレッドから安全に参照可能な状態照会
    bool is_connected() const { return connected_.load(); }
    const std::string &get_port() const { return port_; }

private:
    void update();
    void tx_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    bool try_open_port();
    void close_port();
    void maybe_log_status();

    int fd_;
    std::atomic<bool> connected_{false};
    uint8_t device_id_;
    std::string port_;
    std::chrono::steady_clock::duration reconnect_interval_;
    std::chrono::steady_clock::duration rx_timeout_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_;
    std::chrono::steady_clock::time_point last_rx_time_;
    std::chrono::steady_clock::time_point last_status_log_time_;
    std::chrono::milliseconds status_log_period_{500};

    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<uint8_t> rx_buffer_;

    bool verbose_packet_log_{false};
    uint64_t rx_frames_since_status_{0};
    uint64_t rx_bytes_since_status_{0};
    uint64_t tx_bytes_since_status_{0};
    uint64_t rx_total_bytes_{0};
    uint64_t tx_total_bytes_{0};
    uint64_t dropped_bytes_since_status_{0};
    uint64_t checksum_errors_since_status_{0};
    uint64_t id_mismatch_since_status_{0};
    uint64_t tx_frames_since_status_{0};
    uint64_t tx_errors_since_status_{0};

    // pub,sub
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tx_sub_;
};
