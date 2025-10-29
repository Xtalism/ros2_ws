#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <fmt/core.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

static std::atomic<bool> g_interrupt_requested{false};
using std::placeholders::_1;
// inline const char* ip_address = "192.168.10.1";
inline const char* ip_address = "192.168.100.204";

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

void signal_handler(int signum) {
    if (signum == SIGINT) {
        YASMIN_LOG_WARN("SIGINT received - initiating safe shutdown...");
        g_interrupt_requested = true;
    }
}

class YasminRos {
public:
    YasminRos(std::shared_ptr<rclcpp::Node> node)
        : node_(node), battery_percentage_(100.0)
    {
        pub_takeoff_ = node_->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
        pub_control_ = node_->create_publisher<geometry_msgs::msg::Twist>("control", 10);
        pub_land_    = node_->create_publisher<std_msgs::msg::Empty>("land", 10);
        sub_battery_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                battery_percentage_ = msg->percentage;
                YASMIN_LOG_INFO("Battery Percentage: %.2f", battery_percentage_);
            });
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
    double get_battery_percentage() const { return battery_percentage_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
    double battery_percentage_;
};

class StateHandler : public yasmin::State {
    public:
        using yasmin::State::State;

        std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
            if (g_interrupt_requested) {
                YASMIN_LOG_WARN("CB: Interrupt detected...");
                return "keyinterrupt";
            }
            return execute_impl(blackboard);
        }
        virtual std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) = 0;
};

class PingState : public StateHandler {
public:
    PingState(std::shared_ptr<YasminRos> ros)
        : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::TIMEOUT, yasmin_ros::basic_outcomes::RETRY, "keyinterrupt"}), ros_(ros) {}

    std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        int attempts = 0;
        const int max_attempts = 3;
        while (attempts < max_attempts) {
            int handshake = system(fmt::format("ping -c2 -s1 {} > /dev/null 2>&1", ip_address).c_str());
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

class TakeoffState : public StateHandler {
public:
    TakeoffState(std::shared_ptr<YasminRos> ros)
        : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, "keyinterrupt"}), ros_(ros) {}

    std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        YASMIN_LOG_INFO("CB: takeoff... waiting for subscriber");
        bool ok = YasminRos::wait_for_subscribers<std_msgs::msg::Empty>(ros_->get_takeoff_pub(), 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /takeoff within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /takeoff", ros_->get_takeoff_pub()->get_subscription_count());
        }
        std_msgs::msg::Empty msg;
        ros_->get_takeoff_pub()->publish(msg);

        if (!interruptible_sleep(std::chrono::seconds(10))) {
            return "keyinterrupt";
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }
private:
    std::shared_ptr<YasminRos> ros_;
};

class TwistLState : public StateHandler {
    public:
        TwistLState(std::shared_ptr<YasminRos> ros)
            : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, "keyinterrupt"}), ros_(ros) {}

        std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
            YASMIN_LOG_INFO("CB: yaw left...");
            bool ok = YasminRos::wait_for_subscribers<geometry_msgs::msg::Twist>(ros_->get_control_pub(), 1, std::chrono::milliseconds(3000));
            if (!ok) {
                YASMIN_LOG_WARN("No subscriber on /control within 3s; publishing anyway");
            } else {
                YASMIN_LOG_INFO("Matched %zu subscriber(s) on /control", ros_->get_control_pub()->get_subscription_count());
            }
            geometry_msgs::msg::Twist msg;
            msg.angular.z = -50.0;
            ros_->get_control_pub()->publish(msg);

            if (!interruptible_sleep(std::chrono::seconds(10))) {
                return "keyinterrupt";
            }
            return yasmin_ros::basic_outcomes::SUCCEED;
        }
    private:
        std::shared_ptr<YasminRos> ros_;
};

class TwistRState : public StateHandler {
    public:
        TwistRState(std::shared_ptr<YasminRos> ros)
            : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, "keyinterrupt"}), ros_(ros) {}

        std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
            YASMIN_LOG_INFO("CB: yaw left...");
            bool ok = YasminRos::wait_for_subscribers<geometry_msgs::msg::Twist>(ros_->get_control_pub(), 1, std::chrono::milliseconds(3000));
            if (!ok) {
                YASMIN_LOG_WARN("No subscriber on /control within 3s; publishing anyway");
            } else {
                YASMIN_LOG_INFO("Matched %zu subscriber(s) on /control", ros_->get_control_pub()->get_subscription_count());
            }
            geometry_msgs::msg::Twist msg;
            msg.angular.z = 50.0;
            ros_->get_control_pub()->publish(msg);

            if (!interruptible_sleep(std::chrono::seconds(10))) {
                return "keyinterrupt";
            }
            return yasmin_ros::basic_outcomes::SUCCEED;
        }
    private:
        std::shared_ptr<YasminRos> ros_;
};

class RollLState : public StateHandler {
    public:
        RollLState(std::shared_ptr<YasminRos> ros)
            : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, "keyinterrupt"}), ros_(ros) {}

    std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
        YASMIN_LOG_INFO("CB: Roll left...");
        bool ok = YasminRos::wait_for_subscribers<geometry_msgs::msg::Twist>(ros_->get_control_pub(), 1, std::chrono::milliseconds(3000));
        if (!ok) {
            YASMIN_LOG_WARN("No subscriber on /control within 3s; publishing anyway");
        } else {
            YASMIN_LOG_INFO("Matched %zu subscriber(s) on /control", ros_->get_control_pub()->get_subscription_count());
        }
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 50;
        ros_->get_control_pub()->publish(msg);
        
        if (!interruptible_sleep(std::chrono::seconds(10))) {
            return "keyinterrupt";
        }
        return yasmin_ros::basic_outcomes::SUCCEED;
    }
    private:
        std::shared_ptr<YasminRos> ros_;
};

class LandState : public StateHandler {
public:
    LandState(std::shared_ptr<YasminRos> ros)
        : StateHandler({yasmin_ros::basic_outcomes::SUCCEED, "keyinterrupt"}), ros_(ros) {}

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
        return execute_impl(blackboard);
    }

    std::string execute_impl(std::shared_ptr<yasmin::blackboard::Blackboard>) override {
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