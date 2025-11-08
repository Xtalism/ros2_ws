// Include file 
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// C++ includes
#include <iostream> 
#include <algorithm> 
#include <fstream> 
#include <chrono> 
#include <vector> 
#include <queue>
#include <thread> 
#include <mutex> 
#include <cstdlib> 

#include <cstring>
#include <sstream> 

//* ROS2 includes
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;

// Include Eigen
#include <Eigen/Dense> 

// Include cv-bridge
#include <cv_bridge/cv_bridge.hpp>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

//* ORB SLAM 3 includes
#include "System.h" 
#include "ImuTypes.h" 

//* Gobal defs
#define pass (void)0 


//* Node specific definitions
class MonocularMode : public rclcpp::Node
{   
    public:
    std::string experimentConfig = ""; 
    double timeStep; 
    std::string receivedConfig = "";

    //* Class constructor
    MonocularMode(); 

    ~MonocularMode(); 
        
    private:
        
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_ws/src/ros2_orb_slam3/"; 
        std::string OPENCV_WINDOW = ""; 
        std::string nodeName = ""; 
        std::string vocFilePath = ""; 
        std::string settingsFilePath = ""; 
        bool bSettingsFromPython = false; 
        bool useImu = true; // Flag to enable/disable IMU
        
        std::string subexperimentconfigName = ""; 
        std::string pubconfigackName = ""; 
        std::string subImgMsgName = ""; 
        std::string subImuMsgName = ""; 
        std::string subTimestepMsgName = ""; 

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImuMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; 
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; 
        bool enableOpenCVWindow = false; 

        //* IMU-Image synchronization variables
        std::mutex imuMutex;
        std::vector<ORB_SLAM3::IMU::Point> imuBuffer;
        cv::Mat currentImage;
        double currentImageTime;
        double lastImageTime;
        bool imageReceived;
        bool imuInitialized;
        size_t minImuMeasurements; // Minimum IMU measurements before processing first image

        //* ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); 
        void Timestep_callback(const std_msgs::msg::Float64& time_msg); 
        void Img_callback(const sensor_msgs::msg::Image::SharedPtr msg); 
        void Imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); 
        
        //* Helper functions
        void ProcessImageWithIMU(); 
        void initializeVSLAM(std::string& configString); 


};

#endif