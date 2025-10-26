#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

static std::atomic<bool> g_interrupt_requested{false};

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

class YasminRos {
public:
    explicit YasminRos(std::shared_ptr<rclcpp::Node> node)
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

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_takeoff_pub() const { return pub_takeoff_; }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr get_control_pub() const { return pub_control_; }
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_land_pub() const { return pub_land_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
};

class PingState : public yasmin::State {
public:
    PingState(std::shared_ptr<YasminRos> ros)
        : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::TIMEOUT}), ros_(ros) {}

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        int attempts = 0;
        const int max_attempts = 3;
        while (attempts < max_attempts) {
            int handshake = system("ping -c2 -s1 192.168.10.1 > /dev/null 2>&1");
            if (handshake == 0) {
                YASMIN_LOG_INFO("Ping success");
                return yasmin_ros::basic_outcomes::SUCCEED;
            } else {
                ++attempts;
                if (attempts < max_attempts) {
                    YASMIN_LOG_WARN("Ping failed, retrying... (attempt %d/%d)", attempts, max_attempts);
                }
            }
        }
        YASMIN_LOG_WARN("Ping failed after %d attempts, giving up.", max_attempts);
        return yasmin_ros::basic_outcomes::TIMEOUT;
    }
private:
    std::shared_ptr<YasminRos> ros_;
};

class TakeoffState : public yasmin::State {
public:
    TakeoffState(std::shared_ptr<YasminRos> ros)
        : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}), ros_(ros) {}

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        YASMIN_LOG_INFO("CB: takeoff... waiting for subscriber");
        bool ok = YasminRos::wait_for_subscribers<std_msgs::msg::Empty>(ros_->get_takeoff_pub(), 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /takeoff within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /takeoff", ros_->get_takeoff_pub()->get_subscription_count());
        }
        std_msgs::msg::Empty msg;
        ros_->get_takeoff_pub()->publish(msg);

        if (!interruptible_sleep(std::chrono::seconds(2))) {
            return yasmin_ros::basic_outcomes::TIMEOUT;
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }
private:
    std::shared_ptr<YasminRos> ros_;
};

class LandState : public yasmin::State {
public:
    LandState(std::shared_ptr<YasminRos> ros)
        : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}), ros_(ros) {}

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        YASMIN_LOG_INFO("CB: land... waiting for subscriber");
        bool ok = YasminRos::wait_for_subscribers<std_msgs::msg::Empty>(ros_->get_land_pub(), 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /land within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /land", ros_->get_land_pub()->get_subscription_count());
        }
        std_msgs::msg::Empty msg;
        ros_->get_land_pub()->publish(msg);

        YASMIN_LOG_INFO("Landing in progress - waiting for completion...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        YASMIN_LOG_INFO("Landing complete");
        return yasmin_ros::basic_outcomes::SUCCEED;
    }
private:
    std::shared_ptr<YasminRos> ros_;
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
    auto yasmin_ros = std::make_shared<YasminRos>(node);

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::TIMEOUT,
            // yasmin_ros::basic_outcomes::ABORT,
        });

    sm->add_state("PING", std::make_shared<PingState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TAKEOFF"},
            {yasmin_ros::basic_outcomes::TIMEOUT, yasmin_ros::basic_outcomes::TIMEOUT}
        });

    sm->add_state("TAKEOFF", std::make_shared<TakeoffState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "LAND"}
        });

    sm->add_state("LAND", std::make_shared<LandState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED}
        });

    sm->set_start_state("PING");

    yasmin_viewer::YasminViewerPub yasmin_pub("SIMPLE_STATE", sm);
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    while (rclcpp::ok()) {
        YASMIN_LOG_INFO("Starting state machine execution...");
        std::string outcome = (*sm.get())(blackboard);
        YASMIN_LOG_INFO("State machine finished with outcome: %s", outcome.c_str());

        if (outcome == yasmin_ros::basic_outcomes::TIMEOUT) {
            YASMIN_LOG_WARN("Mission timed out");
        } else if (outcome == yasmin_ros::basic_outcomes::SUCCEED) {
            YASMIN_LOG_INFO("Mission completed successfully");
        } else {
            YASMIN_LOG_WARN("Unknown outcome: %s", outcome.c_str());
        }

        YASMIN_LOG_INFO("Waiting for viewer updates to propagate...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        break; // only run once for this 
    }

    YASMIN_LOG_INFO("Shutting down gracefully...");
    node.reset();
    rclcpp::shutdown();
    return 0;
}