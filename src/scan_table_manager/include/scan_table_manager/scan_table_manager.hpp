#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_set>

// Central controller node for the Scan Table
class ScanTableManager : public rclcpp::Node {
public:
    // Constructor
    explicit ScanTableManager();

private:
    // -------------------------
    // State Machine Definition
    // -------------------------
    enum class State {
        IDLE,           // Waiting for an item to be placed on the table
        SCANNING,       // Barcode scanning in progress
        PUSHING,        // Pusher mechanism executing (POCKET or REJECT)
        WAIT_FOR_CLEAR  // Wait until table is confirmed empty before next item
    };

    State state_;  // Current FSM state

    // -------------------------
    // Internal Status Variables
    // -------------------------
    bool table_occupied_{false};                // Tracks if table currently has an item
    std::unordered_set<std::string> barcodes_; // Stores unique barcodes collected during scan

    // Timer for scanning window
    rclcpp::TimerBase::SharedPtr scan_timer_;

    // -------------------------
    // Publishers
    // -------------------------
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pusher_pub_;   // Command pusher (POCKET/REJECT)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;      // Notify robot table is ready
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;    // Trigger mode for barcode scanner

    // -------------------------
    // Subscribers
    // -------------------------
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr item_sub_;        // Item placed on table
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub_;   // Barcode readings
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr table_clear_sub_; // Table occupancy sensor
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pusher_done_sub_; // Pusher completion notification

    // -------------------------
    // Callback Functions
    // -------------------------
    void onItemPlaced(const std_msgs::msg::Bool::SharedPtr msg);     // Triggered when robot places an item
    void onBarcode(const std_msgs::msg::String::SharedPtr msg);     // Collect barcode data
    void onScanTimeout();                                           // Called when scan window expires
    void onPusherDone(const std_msgs::msg::Bool::SharedPtr msg);    // Called when pusher finishes action
    void onTableClear(const std_msgs::msg::Bool::SharedPtr msg);    // Called when table is confirmed empty

    // -------------------------
    // Helper Functions
    // -------------------------
    void sendPusher(const std::string &cmd); // Send POCKET or REJECT command to pusher
};
