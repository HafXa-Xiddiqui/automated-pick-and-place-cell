#include "scan_table_manager/scan_table_manager.hpp"

using std::placeholders::_1;

ScanTableManager::ScanTableManager()
: Node("scan_table_manager"),
  state_(State::WAIT_FOR_ITEM),
  table_occupied_(false),
  item_placed_(false),
  pusher_done_(false),
  scan_timeout_sec_(2.0)
{
    // Publishers
    trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/scanner/trigger", 10);
    pusher_pub_  = create_publisher<std_msgs::msg::String>("/pusher/cmd", 10);
    ready_pub_   = create_publisher<std_msgs::msg::Bool>("/ready_for_next_item", 10);

    // Subscribers
    item_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/item_placed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg){ item_placed_ = msg->data; });

    barcode_sub_ = create_subscription<std_msgs::msg::String>(
        "/scanner/barcode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){
            collected_barcodes_.push_back(msg->data);
        });

    pusher_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/pusher/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg){
            pusher_done_ = (msg->data == "finished");
        });

    table_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/table/occupied", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            table_occupied_ = msg->data;
        });

    // Main loop timer
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&ScanTableManager::tick, this));

    RCLCPP_INFO(get_logger(), "ScanTableManager started.");
}

void ScanTableManager::enter_state(State s) {
    state_ = s;

    switch (state_) {
        case State::SCAN_ITEM:
            collected_barcodes_.clear();
            scan_start_time_ = now();
            break;

        case State::ACTUATE_PUSHER:
            pusher_done_ = false;
            break;

        default:
            break;
    }
}

void ScanTableManager::tick() {
    switch (state_) {

    case State::WAIT_FOR_ITEM:
        if (table_occupied_) {
            RCLCPP_INFO(get_logger(), "Item detected. Starting scan.");
            enter_state(State::SCAN_ITEM);
        }
        break;

    case State::SCAN_ITEM: {
        std_msgs::msg::Bool trig;
        trig.data = true;
        trigger_pub_->publish(trig);

        auto elapsed = (now() - scan_start_time_).seconds();
        if (elapsed > scan_timeout_sec_) {
            enter_state(State::PROCESS_RESULTS);
        }
        break;
    }

    case State::PROCESS_RESULTS: {
        std::set<std::string> unique_barcodes(collected_barcodes_.begin(),
                                              collected_barcodes_.end());

        bool is_good = !unique_barcodes.empty();

        std_msgs::msg::String cmd;
        cmd.data = is_good ? "push_left" : "push_right";
        pusher_pub_->publish(cmd);

        enter_state(State::ACTUATE_PUSHER);
        break;
    }

    case State::ACTUATE_PUSHER:
        if (pusher_done_) {
            enter_state(State::VERIFY_TABLE_CLEAR);
        }
        break;

    case State::VERIFY_TABLE_CLEAR:
        if (!table_occupied_) {
            enter_state(State::READY_SIGNAL);
        }
        break;

    case State::READY_SIGNAL: {
        std_msgs::msg::Bool msg;
        msg.data = true;
        ready_pub_->publish(msg);

        enter_state(State::WAIT_FOR_ITEM);
        break;
    }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanTableManager>());
    rclcpp::shutdown();
    return 0;
}
