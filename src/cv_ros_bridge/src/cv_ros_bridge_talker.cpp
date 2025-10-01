#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("opencv_image_talker")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_cv_bridge", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ImagePublisher::timer_callback, this));

    // Generate random color
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    color_ = cv::Scalar(dis(gen), dis(gen), dis(gen));

    // Generate image
    img_ = cv::Mat(300, 300, CV_8UC3, color_);
  }

private:
  void timer_callback()
  {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img_).toImageMsg();
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Publishing Image");
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat img_;
  cv::Scalar color_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
