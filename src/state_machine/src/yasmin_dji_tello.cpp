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
        pub_ping_    = node_->create_publisher<std_msgs::msg::Empty>("ping_success", 10);
        pub_takeoff_ = node_->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
        pub_control_ = node_->create_publisher<geometry_msgs::msg::Twist>("control", 10);
        pub_land_    = node_->create_publisher<std_msgs::msg::Empty>("land", 10);
    }

    std::string ping_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: pinging...");
        std_msgs::msg::Empty msg;
        pub_ping_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Custom abort logic here if needed
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string takeoff_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: takeoff...");
        std_msgs::msg::Empty msg;
        pub_takeoff_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string yaw_left_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: yaw left...");
        geometry_msgs::msg::Twist msg;
        msg.angular.z = -50.0;
        pub_control_->publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string yaw_right_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: yaw right...");
        geometry_msgs::msg::Twist msg;
        msg.angular.z = 50.0;
        pub_control_->publish(msg);
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

    std::string aborted_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_WARN("FSM Aborted.");
        return yasmin_ros::basic_outcomes::ABORT;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_ping_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
};

int main(int argc, char *argv[]) {
    YASMIN_LOG_INFO("yasmin_dji_tello");
    rclcpp::init(argc, argv);

    // yasmin_ros::set_loggers();

    auto node = std::make_shared<rclcpp::Node>("yasmin_cbstate_helper");
    auto helper = std::make_shared<YasminDroneStateHelper>(node);

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::FAIL,
            yasmin_ros::basic_outcomes::ABORT,
        });

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running()) {
            sm->cancel_state();
        }
    });

    sm->add_state("PING_CONNECTION",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::FAIL,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::ping_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "VERIFY_SENSORS"},
            {yasmin_ros::basic_outcomes::FAIL, "TIMEOUT"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
        });
    sm->add_state("VERIFY_SENSORS",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::FAIL,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::ping_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TAKEOFF"},
            {yasmin_ros::basic_outcomes::FAIL, "NO_TOPICS_DATA"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
        });

    sm->add_state("TAKEOFF",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
                yasmin_ros::basic_outcomes::FAIL,
            },
            std::bind(&YasminDroneStateHelper::takeoff_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "YAW_LEFT"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
            {yasmin_ros::basic_outcomes::FAIL, "NO_TOPICS_DATA"},
        });

    sm->add_state("YAW_LEFT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
                yasmin_ros::basic_outcomes::FAIL,
            },
            std::bind(&YasminDroneStateHelper::yaw_left_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "YAW_RIGHT"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
            {yasmin_ros::basic_outcomes::FAIL, "NO_TOPICS_DATA"},
        });

    sm->add_state("YAW_RIGHT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
                yasmin_ros::basic_outcomes::FAIL,
            },
            std::bind(&YasminDroneStateHelper::yaw_right_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "LANDING"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
            {yasmin_ros::basic_outcomes::FAIL, "NO_TOPICS_DATA"},
        });

    sm->add_state("LANDING",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
                yasmin_ros::basic_outcomes::FAIL,
            },
            std::bind(&YasminDroneStateHelper::land_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED},
            {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
        });

    sm->add_state("TIMEOUT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{yasmin_ros::basic_outcomes::FAIL},
            std::bind(&YasminDroneStateHelper::aborted_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::FAIL, "LANDING"}
        });
    sm->add_state("NO_TOPICS_DATA",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{yasmin_ros::basic_outcomes::FAIL},
            std::bind(&YasminDroneStateHelper::aborted_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::FAIL, "LANDING"}
        });
    sm->add_state("KEYBOARD_INTERRUPT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{yasmin_ros::basic_outcomes::ABORT},
            std::bind(&YasminDroneStateHelper::aborted_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::ABORT, "LANDING"},
        });

    yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_DJI_TELLO", sm);

    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
        std::make_shared<yasmin::blackboard::Blackboard>();

    try {
        std::string outcome = (*sm.get())(blackboard);
        YASMIN_LOG_INFO(outcome.c_str());
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
