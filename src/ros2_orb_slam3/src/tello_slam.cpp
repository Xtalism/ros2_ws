//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("tello_slam_node")
{
    homeDir = getenv("HOME");
    
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");
    this->declare_parameter("config_name", "DJI_Tello"); // Add parameter for config file name
    
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("config_name");
    std::string configName = param4.as_string();
    
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular-Inertial/";
    }
    
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    
    // Direct subscription to Tello camera topic
    subImgMsgName = "/camera"; // Tello publishes to /camera (see your launch file remapping!)
    
    //* Subscribe directly to the Tello image stream
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 10, std::bind(&MonocularMode::Img_callback, this, _1));

    // Initialize SLAM system immediately with config name
    std::string fullConfigPath = settingsFilePath + configName + ".yaml";
    
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullConfigPath.c_str());
    
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true;
    enableOpenCVWindow = false; // Set to true if you want to see images
    
    pAgent = new ORB_SLAM3::System(vocFilePath, fullConfigPath, sensorType, enablePangolinWindow);
    
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized and ready for Tello images");
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    pAgent->Shutdown();
}

//* Callback to process image message and run SLAM node
// void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
void MonocularMode::Img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
        return;
    }
    
    // Use image timestamp from ROS message
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    //* Perform ORB-SLAM3 tracking
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp);
}