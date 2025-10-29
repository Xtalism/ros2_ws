#include <csignal>
#include "dji_tello_states.hpp"

int main(int argc, char *argv[]) {
    YASMIN_LOG_INFO("SIMPLE_STATE");
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ROS2_Subscriber>());

    std::signal(SIGINT, signal_handler);
    yasmin_ros::set_ros_loggers();

    auto node = std::make_shared<rclcpp::Node>("yasmin_cbstate_helper");
    auto yasmin_ros = std::make_shared<YasminRos>(node);

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::TIMEOUT,
            yasmin_ros::basic_outcomes::ABORT,
        });

    sm->add_state("PING", std::make_shared<PingState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TAKEOFF"},
            {yasmin_ros::basic_outcomes::TIMEOUT, yasmin_ros::basic_outcomes::TIMEOUT},
            {yasmin_ros::basic_outcomes::RETRY, "PING"},
            {"keyinterrupt", yasmin_ros::basic_outcomes::ABORT},
        });

    sm->add_state("TAKEOFF", std::make_shared<TakeoffState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TWIST_LEFT"},
            {"keyinterrupt", "LAND"}
        });
    
    sm->add_state("TWIST_LEFT", std::make_shared<TwistLState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "TWIST_RIGHT"},
            {"keyinterrupt", "LAND"}
        });

    sm->add_state("TWIST_RIGHT", std::make_shared<TwistRState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "ROLL_LEFT"},
            {"keyinterrupt", "LAND"}
        });

    sm->add_state("ROLL_LEFT", std::make_shared<RollLState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "LAND"},
            {"keyinterrupt", "LAND"}
        });

    sm->add_state("LAND", std::make_shared<LandState>(yasmin_ros),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED},
            {"keyinterrupt", yasmin_ros::basic_outcomes::ABORT}
        });

    sm->set_start_state("PING");

    yasmin_viewer::YasminViewerPub yasmin_pub("SIMPLE_STATE", sm);
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        try {
            YASMIN_LOG_INFO("Starting state machine execution...");
            std::string outcome = (*sm.get())(blackboard);
            YASMIN_LOG_INFO("State machine finished with outcome: %s", outcome.c_str());
            
            if (outcome == "keyinterrupt") {
                YASMIN_LOG_WARN("Mission aborted - emergency landing completed");
            } else if (outcome == yasmin_ros::basic_outcomes::TIMEOUT) {
                YASMIN_LOG_WARN("Mission timed out");
            } else {
                YASMIN_LOG_INFO("Mission completed successfully");
            }

            YASMIN_LOG_INFO("Waiting for viewer updates to propagate...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break; 
        } catch (const std::exception &e) {
            YASMIN_LOG_ERROR("Exception caught: %s", e.what());
            YASMIN_LOG_WARN("Attempting emergency landing...");
            yasmin_ros->get_land_pub()->publish(std_msgs::msg::Empty());
    }

    YASMIN_LOG_INFO("Shutting down gracefully...");
    node.reset();
    rclcpp::shutdown();
    return 0;
    }
}