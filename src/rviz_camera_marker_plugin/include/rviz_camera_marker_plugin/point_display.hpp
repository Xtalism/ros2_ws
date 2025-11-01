#ifndef RVIZ_CAMERA_MARKER_PLUGIN__POINT_DISPLAY_HPP_
#define RVIZ_CAMERA_MARKER_PLUGIN__POINT_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <rviz_camera_marker_msg/msg/point2_d.hpp>

namespace rviz_camera_marker_plugin
{
class PointDisplay
    : public rviz_common::MessageFilterDisplay<rviz_camera_marker_msg::msg::Point2D>
{
    Q_OBJECT

protected:
    void processMessage(const rviz_camera_marker_msg::msg::Point2D::ConstSharedPtr msg) override;
};
}

#endif