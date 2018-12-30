#include "mappr_rviz/location_array_display.h"

#include "rviz/properties/bool_property.h"
#include <QString>
#include <memory>

namespace mappr
{
namespace viz
{
LocationArrayDisplay::LocationArrayDisplay()
{
  showLabels_ = new rviz::BoolProperty("show Labels", true, "Draw the Labels.", this, SLOT(slotShowLabels()));
}

void LocationArrayDisplay::onInitialize()
{
  locationVis_ = std::make_unique<LocationVisual>(context_->getSceneManager(), scene_node_);

  initialized = false;
}

void LocationArrayDisplay::reset()
{
  initialized = false;
}

void LocationArrayDisplay::firstMessage(const mappr_msgs::LocationArray::ConstPtr& msg)
{
  initMsg_ = msg;

  initialized = true;
}

void LocationArrayDisplay::processMessage(const mappr_msgs::LocationArray::ConstPtr& msg)
{
  if (!initialized)
  {
    firstMessage(msg);
  }
}

void LocationArrayDisplay::slotShowLabels()
{
  ROS_DEBUG_STREAM("show labels: " << ((showLabels_->getBool()) ? "true" : "false"));
}

}  // namespace viz
}  // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::LocationArrayDisplay, rviz::Display)