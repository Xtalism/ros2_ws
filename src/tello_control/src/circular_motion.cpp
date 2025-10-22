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
        /**
         * Timer for circular trajectory execution.
         */
        rclcpp::TimerBase::SharedPtr trajectory_timer;

        /**
         * Current angle in the circular trajectory (radians).
         */
        double theta = 0.0;

        /**
         * Number of points in the circle (matching your matplotlib code: 20).
         */
        int num_points = 20;

        /**
         * Current point index in trajectory.
         */
        int current_point = 0;

        /**
         * Radius of the circle in cm.
         */
        double radius = 50.0;  // 5 units from your Python code, scaled to cm

        /**
         * Center of the circle.
         */
        double x_center = 50.0;
        double y_center = 20.0;
        double z_center = 0.0;

        /**
         * Time to spend at each point (seconds).
         */
        double time_per_point = 2.0;

        /**
         * Speed for movement between points.
         */
        double movement_speed = 30.0;

        /**
         * State machine for autonomous execution.
         */
        enum State {
            IDLE,
            TAKING_OFF,
            WAITING_TAKEOFF,
            EXECUTING_TRAJECTORY,
            TRAJECTORY_COMPLETE,
            LANDING
        };
        State current_state = IDLE;

        /**
         * Timer for state transitions.
         */
        rclcpp::Time state_start_time;

        /**
         * Publish drone control velocity.
         */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;

        /**
         * Publish takeoff control.
         */
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_takeoff;

        /**
         * Publisher for landing controls.
         */
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_land;

        /**
         * Publisher for emergency stop.
         */
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_emergency;

        /**
         * Construct a new Tello Control object
         */
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
            RCLCPP_INFO(this->get_logger(), "  - Radius: %.1f cm", radius);
            RCLCPP_INFO(this->get_logger(), "  - Center: (%.1f, %.1f, %.1f)", x_center, y_center, z_center);
            RCLCPP_INFO(this->get_logger(), "  - Points: %d", num_points);
            RCLCPP_INFO(this->get_logger(), "Starting autonomous sequence in 3 seconds...");
            
            // Start the sequence after 3 seconds
            auto start_timer = this->create_wall_timer(
                3s, [this]() { this->startSequence(); });
        }

        /**
         * Start the autonomous sequence.
         */
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

        /**
         * Main autonomous control callback.
         */
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
                        RCLCPP_INFO(this->get_logger(), "Executing %d points on XY plane...", num_points);
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

                    // Get current target point
                    double target_x, target_y, target_z;
                    getTrajectoryPoint(current_point, target_x, target_y, target_z);

                    // Calculate time-based progression
                    double progress = elapsed / time_per_point;
                    
                    if (progress >= 1.0) {
                        // Move to next point
                        current_point++;
                        state_start_time = current_time;
                        
                        if (current_point < num_points) {
                            getTrajectoryPoint(current_point, target_x, target_y, target_z);
                            RCLCPP_INFO(this->get_logger(), 
                                "Moving to point %d/%d: (%.1f, %.1f, %.1f)", 
                                current_point + 1, num_points, target_x, target_y, target_z);
                        }
                    } else {
                        // Continue moving towards current point
                        // Use smooth velocity profile
                        double velocity_factor = std::sin(progress * PI);  // Smooth acceleration/deceleration
                        
                        double angle = (2.0 * PI * current_point) / num_points;
                        
                        // Tangential velocity for smooth circular motion
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
