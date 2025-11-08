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
    this->declare_parameter("use_imu", true); // Parameter to enable/disable IMU
    
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
    
    rclcpp::Parameter param5 = this->get_parameter("use_imu");
    useImu = param5.as_bool();
    
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        if (useImu)
        {
            settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular-Inertial/";
        }
        else
        {
            settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Using IMU: %s", useImu ? "YES" : "NO");
    
    // Direct subscription to Tello camera topic
    subImgMsgName = "/camera"; // Tello publishes to /camera (see your launch file remapping!)
    
    // Initialize synchronization variables BEFORE creating subscriptions
    lastImageTime = 0.0;
    imageReceived = false;
    imuInitialized = false;
    minImuMeasurements = 50; // Require at least 50 IMU measurements before first image
    
    //* Subscribe directly to the Tello image stream
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 10, std::bind(&MonocularMode::Img_callback, this, _1));

    if (useImu)
    {
        subImuMsgName = "/imu/data"; // Filtered IMU data with Madgwick filter
        
        //* Subscribe to filtered IMU data
        subImuMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            subImuMsgName, 100, std::bind(&MonocularMode::Imu_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Waiting for IMU initialization (need %d measurements)...", minImuMeasurements);
    }

    // Initialize SLAM system immediately with config name
    std::string fullConfigPath = settingsFilePath + configName + ".yaml";
    
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullConfigPath.c_str());
    
    sensorType = useImu ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = false; // Set to true if you want to see images
    
    pAgent = new ORB_SLAM3::System(vocFilePath, fullConfigPath, sensorType, enablePangolinWindow);
    
    if (useImu)
    {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized in IMU_MONOCULAR mode");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized in MONOCULAR mode");
    }
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    // Clean up resources
    {
        std::lock_guard<std::mutex> lock(imuMutex);
        imuBuffer.clear();
    }
    
    if (pAgent)
    {
        pAgent->Shutdown();
        delete pAgent;
        pAgent = nullptr;
    }
}

//* Callback to process IMU message
void MonocularMode::Imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!useImu) return;
    
    std::lock_guard<std::mutex> lock(imuMutex);
    
    // Convert ROS IMU message to ORB-SLAM3 IMU data
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    cv::Point3f acc(msg->linear_acceleration.x, 
                    msg->linear_acceleration.y, 
                    msg->linear_acceleration.z);
    
    cv::Point3f gyr(msg->angular_velocity.x, 
                    msg->angular_velocity.y, 
                    msg->angular_velocity.z);
    
    // Store IMU measurement
    imuBuffer.push_back(ORB_SLAM3::IMU::Point(acc, gyr, timestamp));
    
    // Check if we have enough measurements for initialization
    if (!imuInitialized && imuBuffer.size() >= minImuMeasurements)
    {
        imuInitialized = true;
        RCLCPP_INFO(this->get_logger(), "IMU initialized with %zu measurements. Ready to process images.", imuBuffer.size());
    }
    
    // Keep buffer size reasonable (e.g., last 1000 measurements)
    if (imuBuffer.size() > 1000)
    {
        imuBuffer.erase(imuBuffer.begin());
    }
    
    // Process image if we have one waiting and IMU is initialized
    if (imageReceived && imuInitialized && !imuBuffer.empty())
    {
        ProcessImageWithIMU();
    }
}

//* Callback to process image message
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
    
    // If not using IMU, process immediately
    if (!useImu)
    {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        try
        {
            Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 tracking failed: %s", e.what());
        }
        return;
    }
    
    // Using IMU - synchronize
    std::lock_guard<std::mutex> lock(imuMutex);
    
    // Check if IMU is initialized
    if (!imuInitialized)
    {
        static int skipCount = 0;
        if (skipCount++ % 30 == 0) // Log every 30 frames
        {
            RCLCPP_WARN(this->get_logger(), "Skipping image - waiting for IMU initialization (%zu/%d measurements)", 
                        imuBuffer.size(), minImuMeasurements);
        }
        return;
    }
    
    // Store current image
    currentImage = cv_ptr->image.clone();
    currentImageTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imageReceived = true;
    
    // Process if we have IMU data
    if (!imuBuffer.empty())
    {
        ProcessImageWithIMU();
    }
}

//* Process image with synchronized IMU data
void MonocularMode::ProcessImageWithIMU()
{
    if (currentImage.empty())
        return;
    
    // Get IMU measurements between last and current image
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    
    // For first frame or when we have IMU data
    if (lastImageTime > 0.0)
    {
        for (auto it = imuBuffer.begin(); it != imuBuffer.end(); )
        {
            if (it->t <= currentImageTime)
            {
                if (it->t >= lastImageTime)
                {
                    vImuMeas.push_back(*it);
                }
                it = imuBuffer.erase(it);
            }
            else
            {
                break; // Keep future measurements
            }
        }
        
        // Log warning if no IMU measurements found
        if (vImuMeas.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No IMU measurements between images (dt: %.3f s)", 
                        currentImageTime - lastImageTime);
        }
    }
    
    try
    {
        //* Perform ORB-SLAM3 tracking with IMU
        Sophus::SE3f Tcw = pAgent->TrackMonocular(currentImage, currentImageTime, vImuMeas);
        
        lastImageTime = currentImageTime;
        imageReceived = false;
        
        // Optional: Log tracking state
        static int frameCount = 0;
        if (frameCount++ % 30 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Processing frame with %zu IMU measurements", vImuMeas.size());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 tracking failed: %s", e.what());
    }
}