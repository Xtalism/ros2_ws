#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

static std::atomic<bool> g_interrupt_requested{false};
class YasminDroneStateHelper {
public:
    explicit YasminDroneStateHelper(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
    {
        pub_takeoff_ = node_->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
        pub_control_ = node_->create_publisher<geometry_msgs::msg::Twist>("control", 10);
        pub_land_    = node_->create_publisher<std_msgs::msg::Empty>("land", 10);
    }

    template<typename MessageType>
    static bool wait_for_subscribers(
        const typename rclcpp::Publisher<MessageType>::SharedPtr& pub,
        size_t min_subs,
        std::chrono::milliseconds timeout)
    {
        auto start = std::chrono::steady_clock::now();
        while (pub->get_subscription_count() < min_subs) {
            if (std::chrono::steady_clock::now() - start > timeout) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    static bool interruptible_sleep(std::chrono::seconds duration) {
        auto end_time = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end_time) {
            if (g_interrupt_requested) {
                YASMIN_LOG_WARN("Interrupt detected during sleep!");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return true; 
    }

    std::string keyboard_interrupt_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_WARN("CB: keyboard interrupt - initiating emergency landing sequence...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return yasmin_ros::basic_outcomes::ABORT;
    }

    std::string takeoff_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: takeoff... waiting for subscriber");
        bool ok = wait_for_subscribers<std_msgs::msg::Empty>(pub_takeoff_, 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /takeoff within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /takeoff", pub_takeoff_->get_subscription_count());
        }

        std_msgs::msg::Empty msg;
        pub_takeoff_->publish(msg);

        if (!interruptible_sleep(std::chrono::seconds(10))) {
            return yasmin_ros::basic_outcomes::ABORT;
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string twist_left_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: yaw left...");
        bool ok = wait_for_subscribers<geometry_msgs::msg::Twist>(pub_control_, 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /control within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /control", pub_control_->get_subscription_count());
        }
        geometry_msgs::msg::Twist msg;
        msg.angular.z = -50.0;
        pub_control_->publish(msg);

        if (!interruptible_sleep(std::chrono::seconds(10))) {
            return yasmin_ros::basic_outcomes::ABORT;
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string twist_right_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: yaw right...");
        bool ok = wait_for_subscribers<geometry_msgs::msg::Twist>(pub_control_, 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /control within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /control", pub_control_->get_subscription_count());
        }
        geometry_msgs::msg::Twist msg;
        msg.angular.z = 50.0;
        pub_control_->publish(msg);

        if (!interruptible_sleep(std::chrono::seconds(10))) {
            return yasmin_ros::basic_outcomes::ABORT;
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    std::string land_cb(std::shared_ptr<yasmin::blackboard::Blackboard>) {
        YASMIN_LOG_INFO("CB: land... waiting for subscriber");
        bool ok = wait_for_subscribers<std_msgs::msg::Empty>(pub_land_, 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /land within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /land", pub_land_->get_subscription_count());
        }

        std_msgs::msg::Empty msg;
        pub_land_->publish(msg);

        YASMIN_LOG_INFO("Landing in progress - waiting for completion...");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        YASMIN_LOG_INFO("Landing complete");
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_ping_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
};

void signal_handler(int signum) {
    if (signum == SIGINT) {
        YASMIN_LOG_WARN("SIGINT received - initiating safe shutdown...");
        g_interrupt_requested = true;
    }
}

int main(int argc, char *argv[]) {
    YASMIN_LOG_INFO("SIMPLE_STATE");
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);

    yasmin_ros::set_ros_loggers();

    auto node = std::make_shared<rclcpp::Node>("yasmin_cbstate_helper");
    auto helper = std::make_shared<YasminDroneStateHelper>(node);

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
        });

    sm->add_state("KEYBOARD_INTERRUPT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::ABORT,
            },
        std::bind(&YasminDroneStateHelper::keyboard_interrupt_cb, helper, std::placeholders::_1)
    ),
    {
        {yasmin_ros::basic_outcomes::ABORT, "LANDING"},
    });

    sm->add_state("TAKEOFF",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::takeoff_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TWIST_LEFT"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
        });

    sm->add_state("TWIST_LEFT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::twist_left_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TWIST_RIGHT"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
        }
    );

    sm->add_state("TWIST_RIGHT",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::twist_right_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "LANDING"},
            {yasmin_ros::basic_outcomes::ABORT, "KEYBOARD_INTERRUPT"},
        }
    );

    sm->add_state("LANDING",
        std::make_shared<yasmin::CbState>(
            std::initializer_list<std::string>{
                yasmin_ros::basic_outcomes::SUCCEED,
                yasmin_ros::basic_outcomes::ABORT,
            },
            std::bind(&YasminDroneStateHelper::land_cb, helper, std::placeholders::_1)
        ),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED},
            {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
        });

    sm->set_start_state("TAKEOFF");

    yasmin_viewer::YasminViewerPub yasmin_pub("SIMPLE_STATE", sm);
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    try {
        YASMIN_LOG_INFO("Starting state machine execution...");
        std::string outcome = (*sm.get())(blackboard);
        YASMIN_LOG_INFO("State machine finished with outcome: %s", outcome.c_str());
        
        if (outcome == yasmin_ros::basic_outcomes::ABORT) {
            YASMIN_LOG_WARN("Mission aborted - emergency landing completed");
        } else {
            YASMIN_LOG_INFO("Mission completed successfully");
        }
        
        YASMIN_LOG_INFO("Waiting for viewer updates to propagate...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
    } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("Exception caught: %s", e.what());
        YASMIN_LOG_WARN("Attempting emergency landing...");
        helper->land_cb(blackboard);
    }

    YASMIN_LOG_INFO("Shutting down gracefully...");
    helper.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}