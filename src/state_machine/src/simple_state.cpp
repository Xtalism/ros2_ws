#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

class YasminDroneStateHelper {
public:
    explicit YasminDroneStateHelper(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        pub_takeoff_ = node_->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
        pub_control_ = node_->create_publisher<geometry_msgs::msg::Twist>("control", 10);
        pub_land_    = node_->create_publisher<std_msgs::msg::Empty>("land", 10);
    }

    std::string takeoff_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: takeoff...");
        std_msgs::msg::Empty msg;
        pub_takeoff_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string land_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: land...");
        std_msgs::msg::Empty msg;
        pub_land_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_ping_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
};

int main(int argc, char *argv[]) {
    YASMIN_LOG_INFO("SIMPLE_STATE");
    rclcpp::init(argc, argv);

    // yasmin_ros::set_loggers();

    auto node = std::make_shared<rclcpp::Node>("yasmin_cbstate_helper");
    auto helper = std::make_shared<YasminDroneStateHelper>(node);

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{
            yasmin_ros::basic_outcomes::SUCCEED,
        });

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running()) {
            sm->cancel_state();
        }
    });

    sm->add_state("TAKEOFF",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
            },
            std::bind(&YasminDroneStateHelper::takeoff_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "LANDING"},
        });

    sm->add_state("LANDING",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
            },
            std::bind(&YasminDroneStateHelper::land_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED},
        });

    yasmin_viewer::YasminViewerPub yasmin_pub("SIMPLE_STATE", sm);

    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
        std::make_shared<yasmin::blackboard::Blackboard>();

    try {
        while (rclcpp::ok()) {
            std::string outcome = (*sm.get())(blackboard);
            YASMIN_LOG_INFO(outcome.c_str());
        }
    } catch (const std::exception &e) {
        YASMIN_LOG_WARN(e.what());
    }

    // Let node and helper go out of scope BEFORE shutdown
    // (node and its publishers destroyed first)
    helper.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
