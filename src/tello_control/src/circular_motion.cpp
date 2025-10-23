#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.14159265359
#define PI2 PI * 2.0
#define DEG_TO_RAD PI / 180

using namespace std::chrono_literals;

class TelloControl : public rclcpp::Node
{
    public:
        rclcpp::TimerBase::SharedPtr trajectory_timer;
        rclcpp::TimerBase::SharedPtr start_timer;

        double theta = 0.0;

        /**
         * Number of points in the circle (matching your matplotlib code: 20).
         */
        int num_points = 20;
        int current_point = 0;
        double radius = 30.0 / (2.0 * PI);  // 30 cm perimeter → radius ≈ 4.77 cm

        double x_center = 0.0;  // Relative to takeoff position
        double y_center = 0.0;
        double z_center = 0.0;

        double time_per_point = 1.0;
        double movement_speed = 15.0;

        enum State {
            IDLE,
            TAKING_OFF,
            WAITING_TAKEOFF,
            EXECUTING_TRAJECTORY,
            TRAJECTORY_COMPLETE,
            LANDING
        };
        State current_state = IDLE;

        rclcpp::Time state_start_time;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_takeoff;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_land;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_emergency;
        
        TelloControl() : Node("control")
        {
            publisher_land = this->create_publisher<std_msgs::msg::Empty>("land", 1);
            publisher_takeoff = this->create_publisher<std_msgs::msg::Empty>("takeoff", 1);
            publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("control", 1);
            publisher_emergency = this->create_publisher<std_msgs::msg::Empty>("emergency", 1);

            // Main control loop timer - runs at 50Hz
            trajectory_timer = this->create_wall_timer(
                20ms, std::bind(&TelloControl::autonomousCallback, this));

            state_start_time = this->now();

            RCLCPP_INFO(this->get_logger(), "=== Autonomous Tello Trajectory Controller ===");
            RCLCPP_INFO(this->get_logger(), "Circle parameters:");
            RCLCPP_INFO(this->get_logger(), "  - Perimeter: 30.00 cm (target)");
            RCLCPP_INFO(this->get_logger(), "  - Radius: %.2f cm", radius);
            RCLCPP_INFO(this->get_logger(), "  - Diameter: %.2f cm", radius * 2.0);
            RCLCPP_INFO(this->get_logger(), "  - Calculated Perimeter: %.2f cm", 2.0 * PI * radius);
            RCLCPP_INFO(this->get_logger(), "  - Center: (%.1f, %.1f, %.1f)", x_center, y_center, z_center);
            RCLCPP_INFO(this->get_logger(), "  - Points: %d", num_points);
            RCLCPP_INFO(this->get_logger(), "Starting autonomous sequence in 3 seconds...");
            
            // Start the sequence after 3 seconds (one-shot timer)
            start_timer = this->create_wall_timer(
                3s, [this]() { 
                    this->startSequence(); 
                    this->start_timer->cancel();  // Cancel after first execution
                });
        }

        void startSequence()
        {
            RCLCPP_INFO(this->get_logger(), "Starting takeoff...");
            current_state = TAKING_OFF;
            state_start_time = this->now();
            publisher_takeoff->publish(std_msgs::msg::Empty());
        }

        /**
         * Calculate trajectory point on XY plane circle.
         */
        void getTrajectoryPoint(int point_index, double &x, double &y, double &z)
        {
            double angle = (2.0 * PI * point_index) / num_points;
            
            // XY plane circle (horizontal)
            x = radius * std::sin(angle);
            y = radius * std::cos(angle);
            z = z_center;  // Constant altitude
        }

        void autonomousCallback()
        {
            auto current_time = this->now();
            double elapsed = (current_time - state_start_time).seconds();

            geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

            switch(current_state)
            {
                case IDLE:
                    // Do nothing, waiting for start
                    break;

                case TAKING_OFF:
                    // Wait a bit for takeoff command to be sent
                    if (elapsed > 1.0) {
                        current_state = WAITING_TAKEOFF;
                        state_start_time = current_time;
                        RCLCPP_INFO(this->get_logger(), "Takeoff initiated, waiting for drone to stabilize...");
                    }
                    break;

                case WAITING_TAKEOFF:
                    // Wait for drone to complete takeoff and stabilize (typically 5-7 seconds)
                    if (elapsed > 6.0) {
                        current_state = EXECUTING_TRAJECTORY;
                        current_point = 0;
                        state_start_time = current_time;
                        RCLCPP_INFO(this->get_logger(), "Starting circular trajectory execution!");
                        RCLCPP_INFO(this->get_logger(), "Executing %d points on 30 cm perimeter circle...", num_points);
                    }
                    break;

                case EXECUTING_TRAJECTORY:
                {
                    if (current_point >= num_points) {
                        // Trajectory complete
                        current_state = TRAJECTORY_COMPLETE;
                        state_start_time = current_time;
                        // Stop movement
                        publisher_velocity->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "Trajectory complete! Hovering...");
                        break;
                    }

                    // Calculate time-based progression
                    double progress = elapsed / time_per_point;
                    
                    if (progress >= 1.0) {
                        // Move to next point
                        current_point++;
                        state_start_time = current_time;
                        
                        if (current_point < num_points) {
                            double next_x, next_y, next_z;
                            getTrajectoryPoint(current_point, next_x, next_y, next_z);
                            RCLCPP_INFO(this->get_logger(), 
                                "Moving to point %d/%d: (%.2f, %.2f, %.2f)", 
                                current_point + 1, num_points, next_x, next_y, next_z);
                        }
                    } else {
                        // Continue moving towards current point
                        // Use smooth velocity profile
                        double velocity_factor = std::sin(progress * PI);  // Smooth acceleration/deceleration
                        
                        double angle = (2.0 * PI * current_point) / num_points;
                        
                        // Tangential velocity for smooth circular motion
                        // For XY plane: vx points in tangent direction, vy perpendicular
                        msg.linear.x = movement_speed * std::cos(angle) * velocity_factor;
                        msg.linear.y = -movement_speed * std::sin(angle) * velocity_factor;
                        msg.linear.z = 0.0;
                        msg.angular.z = 0.0;
                        
                        publisher_velocity->publish(msg);
                    }
                    break;
                }

                case TRAJECTORY_COMPLETE:
                    // Hover for a bit before landing
                    if (elapsed > 3.0) {
                        current_state = LANDING;
                        state_start_time = current_time;
                        RCLCPP_INFO(this->get_logger(), "Initiating landing...");
                        publisher_land->publish(std_msgs::msg::Empty());
                    }
                    break;

                case LANDING:
                    // Wait for landing to complete
                    if (elapsed > 5.0) {
                        RCLCPP_INFO(this->get_logger(), "Autonomous sequence complete!");
                        RCLCPP_INFO(this->get_logger(), "Shutting down node...");
                        rclcpp::shutdown();
                    }
                    break;
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelloControl>());
    rclcpp::shutdown();
    return 0;
}
