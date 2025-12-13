#include "scan_table_manager/scan_table_manager.hpp"

using namespace std::chrono_literals;

ScanTableManager::ScanTableManager()
: Node("scan_table_manager"), state_(State::IDLE)
{
    RCLCPP_INFO(get_logger(), "ScanTableManager started");

    // Subscribers
    item_sub_ = create_subscription<std_msgs::msg::Bool>(
        "item_placed", 10,
        std::bind(&ScanTableManager::onItemPlaced, this, std::placeholders::_1));

    barcode_sub_ = create_subscription<std_msgs::msg::String>(
        "barcode", 10,
        std::bind(&ScanTableManager::onBarcode, this, std::placeholders::_1));

    table_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "table_is_clear", 10,
        std::bind(&ScanTableManager::onTableClear, this, std::placeholders::_1));

    pusher_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "pusher_done", 10,
        std::bind(&ScanTableManager::onPusherDone, this, std::placeholders::_1));

    // Publishers
    pusher_pub_ = create_publisher<std_msgs::msg::String>("pusher_cmd", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("ready_for_next_item", 10);
    trigger_pub_ = create_publisher<std_msgs::msg::Bool>("scanner_trigger", 10); // triggered mode
}

void ScanTableManager::onItemPlaced(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || state_ != State::IDLE) return;

    RCLCPP_INFO(get_logger(), "Item placed → starting scan");
    table_occupied_ = true;
    barcodes_.clear();
    state_ = State::SCANNING;

    // Trigger scanner in triggered mode
    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = true;
    trigger_pub_->publish(trigger_msg);

    // Start scan timer (2 seconds)
    scan_timer_ = create_wall_timer(
        2s, std::bind(&ScanTableManager::onScanTimeout, this));
}

void ScanTableManager::onBarcode(const std_msgs::msg::String::SharedPtr msg) {
    if (state_ != State::SCANNING) return;

    if (barcodes_.insert(msg->data).second) {
        RCLCPP_INFO(get_logger(), "Collected barcode: %s", msg->data.c_str());
    }
}

void ScanTableManager::onScanTimeout() {
    scan_timer_->cancel();

    // Stop triggered scanner
    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = false;
    trigger_pub_->publish(trigger_msg);

    // Decide destination
    std::string dest = barcodes_.empty() ? "REJECT" : "POCKET";
    RCLCPP_INFO(get_logger(), "Scan complete (%lu barcodes) → %s",
                barcodes_.size(), dest.c_str());

    sendPusher(dest);
    state_ = State::PUSHING;
}

void ScanTableManager::sendPusher(const std::string &cmd) {
    std_msgs::msg::String msg;
    msg.data = cmd;
    pusher_pub_->publish(msg);
}

void ScanTableManager::onPusherDone(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || state_ != State::PUSHING) return;

    RCLCPP_INFO(get_logger(), "Pusher finished");
    state_ = State::WAIT_FOR_CLEAR;
}

void ScanTableManager::onTableClear(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || state_ != State::WAIT_FOR_CLEAR) return;

    table_occupied_ = false;
    state_ = State::IDLE;

    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);

    RCLCPP_INFO(get_logger(), "Table clear → Ready for next item");
}

// ------------------- main -------------------

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanTableManager>());
    rclcpp::shutdown();
}
