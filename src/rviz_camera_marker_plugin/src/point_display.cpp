#include <rviz_camera_marker_plugin/point_display.hpp>
#include <rviz_common/logging.hpp>

namespace rviz_camera_marker_plugin
{
void PointDisplay::processMessage(const rviz_camera_marker_msg::msg::Point2D::ConstSharedPtr msg)
{
    RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_camera_marker_plugin::PointDisplay, rviz_common::Display)