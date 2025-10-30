// Standard
#include <csignal>
#include <iostream>
#include <map>
#include <vector>
#include <numeric>
#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "aruco_detector/msg/aruco_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
// #include "image_transport/image_transport.hpp"

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

template <typename T>
std::string SSTR(const T& x) {
    std::ostringstream oss;
    oss << std::dec << x;
    return oss.str();
}

using std::placeholders::_1;
using namespace std::chrono_literals;

// #define SSTR(x) static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode()
    : Node("aruco_detector")
    {
        // Declare and get parameters
        this->declare_parameter("camera", "/camera/color/image_raw");
        this->declare_parameter("camera_info", "/camera/color/camera_info");
        this->declare_parameter("show_detections", true);
        this->declare_parameter("tf_prefix", "marker");
        this->declare_parameter("marker_size", 0.09f);
        this->declare_parameter("enable_blur", true);
        this->declare_parameter("blur_window_size", 7);
        this->declare_parameter("image_fps", 30);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("num_detected", 50);
        this->declare_parameter("min_prec_value", 80);
        this->declare_parameter("dictionary_name", "DICT_4X4_250");
        this->declare_parameter("aruco_adaptiveThreshWinSizeStep", 4);

        this->get_parameter("camera", rgb_topic_);
        this->get_parameter("camera_info", rgb_info_topic_);
        this->get_parameter("show_detections", show_detections_);
        this->get_parameter("tf_prefix", marker_tf_prefix_);
        this->get_parameter("marker_size", marker_size_);
        this->get_parameter("enable_blur", enable_blur_);
        this->get_parameter("blur_window_size", blur_window_size_);
        this->get_parameter("image_fps", image_fps_);
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);
        this->get_parameter("num_detected", num_detected_);
        this->get_parameter("min_prec_value", min_prec_value_);
        this->get_parameter("dictionary_name", dictionary_name_);
        this->get_parameter("aruco_adaptiveThreshWinSizeStep", aruco_adaptiveThreshWinSizeStep_);

        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->adaptiveThreshWinSizeStep = aruco_adaptiveThreshWinSizeStep_;
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        // Dictionary map
        std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names = {
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
            {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
            {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
            {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
            {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
            {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
            {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
            {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
            {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
            {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
            {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
            {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
            {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
            {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
            {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
            {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
        };
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_names[dictionary_name_]);

        // Publishers
        result_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/result_img", 1);
        tf_list_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_list", 10);
        aruco_info_pub_ = this->create_publisher<aruco_detector::msg::ArucoInfo>("/aruco_list", 10);

        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            rgb_topic_, 10, std::bind(&ArucoDetectorNode::image_callback, this, _1));
        rgb_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            rgb_info_topic_, 10, std::bind(&ArucoDetectorNode::camera_info_callback, this, _1));
        parameter_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/update_params", 10, std::bind(&ArucoDetectorNode::update_params_cb, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Aruco Detector Node started.");
    }

private:
    // Parameters
    std::string rgb_topic_, rgb_info_topic_, marker_tf_prefix_, dictionary_name_;
    bool show_detections_;
    float marker_size_;
    bool enable_blur_;
    int blur_window_size_, image_fps_, image_width_, image_height_, num_detected_, min_prec_value_, aruco_adaptiveThreshWinSizeStep_;

    // Camera model
    bool camera_model_computed_ = false;
    image_geometry::PinholeCameraModel camera_model_;
    cv::Mat distortion_coefficients_;
    cv::Matx33d intrinsic_matrix_;

    // Aruco
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_list_pub_;
    rclcpp::Publisher<aruco_detector::msg::ArucoInfo>::SharedPtr aruco_info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr parameter_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Uncertainty
    std::map<int, std::vector<int>> ids_hashmap_;

    // Helper functions
    double getPrec(const std::vector<int>& ids, int i) {
        auto& current_vector = ids_hashmap_[ids[i]];
        int num_detections = std::accumulate(current_vector.begin(), current_vector.end(), 0);
        return (double)num_detections / num_detected_ * 100;
    }

    tf2::Vector3 cv_vector3d_to_tf_vector3(const cv::Vec3d &vec) {
        return {vec[0], vec[1], vec[2]};
    }

    tf2::Quaternion cv_vector3d_to_tf_quaternion(const cv::Vec3d &rotation_vector) {
        auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
        auto angle = sqrt(ax * ax + ay * ay + az * az);
        auto cosa = cos(angle * 0.5);
        auto sina = sin(angle * 0.5);
        auto qx = ax * sina / angle;
        auto qy = ay * sina / angle;
        auto qz = az * sina / angle;
        auto qw = cosa;
        tf2::Quaternion q;
        q.setValue(qx, qy, qz, qw);
        return q;
    }

    tf2::Transform create_transform(const cv::Vec3d &tvec, const cv::Vec3d &rotation_vector) {
        tf2::Transform transform;
        transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
        transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
        return transform;
    }

    // Callbacks
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (camera_model_computed_) return;
        camera_model_.fromCameraInfo(*msg);
        camera_model_.distortionCoeffs().copyTo(distortion_coefficients_);
        intrinsic_matrix_ = camera_model_.intrinsicMatrix();
        camera_model_computed_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera model is computed");
    }

    void update_params_cb(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        // Implement dynamic parameter update if needed
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
        if (!camera_model_computed_) {
            RCLCPP_INFO(this->get_logger(), "Camera model is not computed yet");
            return;
        }

        std::string frame_id = image_msg->header.frame_id;
        auto cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        cv::Mat display_image = image.clone();

        // Smooth the image to improve detection results
        if (enable_blur_) {
            cv::GaussianBlur(image, image, cv::Size(blur_window_size_, blur_window_size_), 0, 0);
        }

        // Detect the markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_, rejected);

        // publish aruco info:
        aruco_detector::msg::ArucoInfo ar_msg;
        for (size_t i = 0; i < ids.size(); i++) {
            auto& one_corner = corners[i];
            ar_msg.marker_ids.push_back(ids[i]);
            ar_msg.center_x_px.push_back((one_corner[0].x + one_corner[1].x + one_corner[2].x + one_corner[3].x) / 4);
            ar_msg.center_y_px.push_back((one_corner[0].y + one_corner[1].y + one_corner[2].y + one_corner[3].y) / 4);
        }
        ar_msg.header.stamp = this->now();
        ar_msg.header.frame_id = "camera";
        aruco_info_pub_->publish(ar_msg);

        // Show image if no markers are detected
        if (ids.empty()) {
            cv::putText(display_image, "no markers found", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
            if (show_detections_) {
                if (result_img_pub_->get_subscription_count() > 0) {
                    auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_image).toImageMsg();
                    result_img_pub_->publish(*out_msg);
                }
            }
        }

        if (!ids.empty()) {
            // Compute poses of markers
            std::vector<cv::Vec3d> rotation_vectors, translation_vectors;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, intrinsic_matrix_, distortion_coefficients_,
                                                 rotation_vectors, translation_vectors);
            // for (size_t i = 0; i < rotation_vectors.size(); ++i) {
            //     cv::aruco::drawAxis(image, intrinsic_matrix_, distortion_coefficients_,
            //                         rotation_vectors[i], translation_vectors[i], marker_size_ * 0.5f);
            // }

            // Draw marker poses
            if (show_detections_) {
                cv::aruco::drawDetectedMarkers(display_image, corners, ids);
            }
            if (result_img_pub_->get_subscription_count() > 0) {
                cv::putText(display_image, "" + SSTR(image_width_) + "x" + SSTR(image_height_) + "@" + SSTR(image_fps_) + "FPS m. size: " + SSTR(marker_size_) + " m" + " blur: " + SSTR(blur_window_size_), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 0), 2);

                for (size_t i = 0; i < ids.size(); i++) {
                    double prec = getPrec(ids, i);
                    if (prec >= min_prec_value_) {
                        cv::Vec3d distance_z_first = translation_vectors[i];
                        double distance_z = ROUND3(distance_z_first[2]);
                        cv::putText(display_image, "id: " + SSTR(ids[i]) + " z dis: " + SSTR(distance_z) + " m  " + SSTR(ROUND2(prec)) + " %", cv::Point(10, 70 + i * 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, CV_RGB(0, 255, 0), 2);
                        if (result_img_pub_->get_subscription_count() > 0) {
                            auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_image).toImageMsg();
                            result_img_pub_->publish(*out_msg);
                        }
                    }
                }
            }

            // Publish TFs for each of the markers
            auto stamp = this->now();
            tf2_msgs::msg::TFMessage tf_msg_list;
            for (size_t i = 0; i < rotation_vectors.size(); ++i) {
                if (getPrec(ids, i) > min_prec_value_) {
                    auto translation_vector = translation_vectors[i];
                    auto rotation_vector = rotation_vectors[i];
                    auto transform = create_transform(translation_vector, rotation_vector);
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = stamp;
                    tf_msg.header.frame_id = frame_id;
                    std::stringstream ss;
                    ss << marker_tf_prefix_ << ids[i];
                    tf_msg.child_frame_id = ss.str();
                    tf_msg.transform.translation.x = transform.getOrigin().getX();
                    tf_msg.transform.translation.y = transform.getOrigin().getY();
                    tf_msg.transform.translation.z = transform.getOrigin().getZ();
                    tf_msg.transform.rotation.x = transform.getRotation().getX();
                    tf_msg.transform.rotation.y = transform.getRotation().getY();
                    tf_msg.transform.rotation.z = transform.getRotation().getZ();
                    tf_msg.transform.rotation.w = transform.getRotation().getW();
                    tf_msg_list.transforms.push_back(tf_msg);
                    tf_broadcaster_->sendTransform(tf_msg);
                }
            }
            tf_list_pub_->publish(tf_msg_list);
        }

        // Uncertainty logic (hashmap update)
        if (num_detected_ > 0) {
            // Rotate vectors
            for (auto& il : ids_hashmap_) {
                std::rotate(il.second.begin(), il.second.end() - 1, il.second.end());
            }
            // Update existing
            for (auto& it : ids_hashmap_) {
                bool current_id_was_found = false;
                for (size_t j = 0; j < ids.size(); j++) {
                    if ((ids[j] == it.first) && (it.second.size() > 1)) {
                        current_id_was_found = true;
                        ids.erase(ids.begin() + j);
                        break;
                    }
                }
                auto& current_vector = it.second;
                current_vector[0] = current_id_was_found ? 1 : 0;
            }
            // Add new
            for (size_t i = 0; i < ids.size(); i++) {
                std::vector<int> tmpp(num_detected_, 0);
                tmpp[0] = 1;
                ids_hashmap_.insert(std::make_pair(ids[i], tmpp));
            }
            // Hack for empty vectors
            for (auto& itt : ids_hashmap_) {
                if (itt.second.size() == 0) {
                    std::vector<int> tmpe(num_detected_, 0);
                    tmpe[0] = 1;
                    itt.second = tmpe;
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}