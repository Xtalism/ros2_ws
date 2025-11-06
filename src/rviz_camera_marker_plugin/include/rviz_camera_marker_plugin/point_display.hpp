#ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
#define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_camera_marker_msg/msg/point3_d.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace rviz_camera_marker_plugin
{
class PointDisplay
  : public rviz_common::MessageFilterDisplay<rviz_camera_marker_msg::msg::Point3D>
{
  Q_OBJECT

private Q_SLOTS:
  void updateStyle();

protected:
  void onInitialize() override;

  void processMessage(const rviz_camera_marker_msg::msg::Point3D::ConstSharedPtr msg) override;

  std::unique_ptr<rviz_rendering::Shape> point_shape_;
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
};
} 

#endif