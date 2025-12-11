#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <string>
#include <set>

class ScanTableManager : public rclcpp::Node {
public:
    ScanTableManager();

private:
    enum class State {
        WAIT_FOR_ITEM,
        SCAN_ITEM,
        PROCESS_RESULTS,
        ACTUATE_PUSHER,
        VERIFY_TABLE_CLEAR,
        READY_SIGNAL
    };

    // State
    State state_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data buffers
    bool table_occupied_;
    bool item_placed_;
    bool pusher_done_;
    std::vector<std::string> collected_barcodes_;

    // Timing
    rclcpp::Time scan_start_time_;
    double scan_timeout_sec_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pusher_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr item_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pusher_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr table_sub_;

    // Internal logic
    void tick();
    void enter_state(State s);
};
