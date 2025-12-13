#include "scan_table_manager/scan_table_manager.hpp"
#include <chrono>

using namespace std::chrono_literals;  // To use 2s, 500ms, etc. directly

// -------------------------
// Constructor
// -------------------------
ScanTableManager::ScanTableManager()
: Node("scan_table_manager"), state_(State::IDLE)  // Initialize node and start in IDLE state
{
    RCLCPP_INFO(get_logger(), "ScanTableManager started");

    // -------------------------
    // Subscribers
    // -------------------------
    // Listen to robot when an item is placed
    item_sub_ = create_subscription<std_msgs::msg::Bool>(
        "item_placed", 10,
        std::bind(&ScanTableManager::onItemPlaced, this, std::placeholders::_1));

    // Listen to barcode scanner outputs
    barcode_sub_ = create_subscription<std_msgs::msg::String>(
        "barcode", 10,
        std::bind(&ScanTableManager::onBarcode, this, std::placeholders::_1));

    // Listen to table occupancy sensor
    table_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "table_is_clear", 10,
        std::bind(&ScanTableManager::onTableClear, this, std::placeholders::_1));

    // Listen to pusher completion signals
    pusher_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "pusher_done", 10,
        std::bind(&ScanTableManager::onPusherDone, this, std::placeholders::_1));

    // -------------------------
    // Publishers
    // -------------------------
    // Send commands to the pusher mechanism
    pusher_pub_ = create_publisher<std_msgs::msg::String>("pusher_cmd", 10);

    // Notify robot that table is ready for next item
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("ready_for_next_item", 10);

    // Trigger scanners in triggered mode
    trigger_pub_ = create_publisher<std_msgs::msg::Bool>("scanner_trigger", 10);
}

// -------------------------
// Called when robot places an item on table
// -------------------------
void ScanTableManager::onItemPlaced(const std_msgs::msg::Bool::SharedPtr msg) {
    // Ignore if not a valid signal or not in IDLE
    if (!msg->data || state_ != State::IDLE) return;

    RCLCPP_INFO(get_logger(), "Item placed → starting scan");

    table_occupied_ = true;  // Mark table as occupied
    barcodes_.clear();       // Reset previous barcode readings
    state_ = State::SCANNING;  // Move FSM to SCANNING

    // Trigger scanner in triggered mode
    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = true;
    trigger_pub_->publish(trigger_msg);

    // Start timer for scanning window (2 seconds)
    scan_timer_ = create_wall_timer(
        2s, std::bind(&ScanTableManager::onScanTimeout, this));
}

// -------------------------
// Called when a barcode message is received
// -------------------------
void ScanTableManager::onBarcode(const std_msgs::msg::String::SharedPtr msg) {
    // Only process barcodes while scanning
    if (state_ != State::SCANNING) return;

    // Insert barcode into set (deduplicate automatically)
    if (barcodes_.insert(msg->data).second) {
        RCLCPP_INFO(get_logger(), "Collected barcode: %s", msg->data.c_str());
    }
}

// -------------------------
// Called when scanning time expires
// -------------------------
void ScanTableManager::onScanTimeout() {
    if (scan_timer_) {
        scan_timer_->cancel();  // Stop the scan timer
    }

    // Stop scanner (triggered mode)
    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = false;
    trigger_pub_->publish(trigger_msg);

    // Decide whether item goes to POCKET or REJECT
    std::string dest = barcodes_.empty() ? "REJECT" : "POCKET";
    RCLCPP_INFO(get_logger(), "Scan complete (%lu barcodes) → %s",
                barcodes_.size(), dest.c_str());

    // Send command to pusher
    sendPusher(dest);

    state_ = State::PUSHING;  // Move FSM to PUSHING
}

// -------------------------
// Send command to pusher
// -------------------------
void ScanTableManager::sendPusher(const std::string &cmd) {
    std_msgs::msg::String msg;
    msg.data = cmd;
    pusher_pub_->publish(msg);
}

// -------------------------
// Called when pusher completes action
// -------------------------
void ScanTableManager::onPusherDone(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || state_ != State::PUSHING) return;

    RCLCPP_INFO(get_logger(), "Pusher finished");
    state_ = State::WAIT_FOR_CLEAR;  // Wait for table to be empty before next item
}

// -------------------------
// Called when table is confirmed empty
// -------------------------
void ScanTableManager::onTableClear(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || state_ != State::WAIT_FOR_CLEAR) return;

    table_occupied_ = false;  // Mark table as free
    state_ = State::IDLE;     // FSM back to IDLE

    // Notify robot to place next item
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);

    RCLCPP_INFO(get_logger(), "Table clear → Ready for next item");
}

// -------------------------
// Main function
// -------------------------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create node instance and spin
    rclcpp::spin(std::make_shared<ScanTableManager>());

    rclcpp::shutdown();
    return 0;
}
