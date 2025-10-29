#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using std::placeholders::_1;

class ROS2_Subscriber : public rclcpp::Node
{
    public:
        ROS2_Subscriber() : Node("battery_percentage")
        {
           subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState> 
           ("battery", 10, std::bind(&ROS2_Subscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const sensor_msgs::msg::BatteryState & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.percentage); 
        }
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROS2_Subscriber>());
    rclcpp::shutdown();
    return 0;
}
