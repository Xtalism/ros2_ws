#include <rviz_camera_marker_plugin/point_display.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/logging.hpp>

namespace rviz_camera_marker_plugin
{
using rviz_common::properties::StatusProperty;

void PointDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_shape_ =
    std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
      scene_node_);

    color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
        "Point Color", QColor(36, 64, 142), "Color to draw the point.", this, SLOT(updateStyle()));
    updateStyle();
}

void PointDisplay::processMessage(const rviz_camera_marker_msg::msg::Point3D::ConstSharedPtr msg)
{
  RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
        "' to frame '" << qPrintable(fixed_frame_) << "'");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->x < 0) {
    setStatus(StatusProperty::Warn, "Message",
        "I will complain about points with negative x values.");
  } else {
    setStatus(StatusProperty::Ok, "Message", "OK");
  }

  Ogre::Vector3 point_pos;
  point_pos.x = msg->x;
  point_pos.y = msg->y;
  point_pos.z = msg->z;
  point_shape_->setPosition(point_pos);
}

void PointDisplay::updateStyle()
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  point_shape_->setColor(color);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_camera_marker_plugin::PointDisplay, rviz_common::Display)