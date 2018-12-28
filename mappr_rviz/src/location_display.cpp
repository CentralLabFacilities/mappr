#include "mappr_rviz/location_display.h"

#include "rviz/properties/bool_property.h"
#include <QString>

namespace mappr {
namespace viz {
LocationDisplay::LocationDisplay() {

  showLabel_ = new rviz::BoolProperty("show Label", true, "Draw the Label.",
                                      this, SLOT(slotShowLabel()));
}

void LocationDisplay::onInitialize() {

  TDClass::onInitialize();

  locationVis_.reset(
      new LocationVisual(context_->getSceneManager(), scene_node_));
  ROS_INFO_STREAM("created location viz");

  initialized = false;
}

void LocationDisplay::reset() {

  TDClass::reset();

  initialized = false;
}

void LocationDisplay::firstMessage(const mappr_msgs::Location::ConstPtr &msg) {
  initMsg_ = msg;

  locationVis_->setMessage(msg);
  initialized = true;
}

void LocationDisplay::processMessage(
    const mappr_msgs::Location::ConstPtr &msg) {

  if (!initialized) {
    firstMessage(msg);
  }
}

void LocationDisplay::slotShowLabel() {
  ROS_DEBUG_STREAM("show label: " << (showLabel_->getBool()) ? "true"
                                                             : "false");
}

} // namespace viz
} // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::LocationDisplay, rviz::Display)