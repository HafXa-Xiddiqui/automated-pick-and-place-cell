#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_set>

class ScanTableManager : public rclcpp::Node {
public:
    explicit ScanTableManager();

private:
    enum class State {
        IDLE,
        SCANNING,
        PUSHING,
        WAIT_FOR_CLEAR
    };

    // FSM state
    State state_;

    // Table & barcode state
    bool table_occupied_{false};
    std::unordered_set<std::string> barcodes_;

    // Timer for scanning window
    rclcpp::TimerBase::SharedPtr scan_timer_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pusher_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_; // For triggered scanners

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr item_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr table_clear_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pusher_done_sub_;

    // Callbacks
    void onItemPlaced(const std_msgs::msg::Bool::SharedPtr msg);
    void onBarcode(const std_msgs::msg::String::SharedPtr msg);
    void onScanTimeout();
    void onPusherDone(const std_msgs::msg::Bool::SharedPtr msg);
    void onTableClear(const std_msgs::msg::Bool::SharedPtr msg);

    // Helper
    void sendPusher(const std::string &cmd);
};
