#include "mappr_rviz/location_array_display.h"

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

#include <QString>
#include <memory>

namespace mappr
{
namespace viz
{
LocationArrayDisplay::LocationArrayDisplay()
{
  showLabels_ = new rviz::BoolProperty("show Labels", true, "Draw the Labels.", this, SLOT(slotShowLabels()));
  labelSize_ = new rviz::FloatProperty("Label size", 1, "Character Height of TextLabel", this, SLOT(slotLabelSize()));
}

void LocationArrayDisplay::onInitialize()
{
  TDClass::onInitialize();
}

void LocationArrayDisplay::reset()
{
  TDClass::reset();
}

QColor boyton[9] = {
  QColor(0, 0, 255),      // Blue
  QColor(255, 0, 0),      // Red
  QColor(0, 255, 0),      // Green
  QColor(255, 255, 0),    // Yellow
  QColor(255, 0, 255),    // Magenta
  QColor(255, 128, 128),  // Pink
  QColor(128, 128, 128),  // Gray
  QColor(128, 0, 0),      // Brown
  QColor(255, 128, 0),    // Orange
};

void LocationArrayDisplay::processMessage(const mappr_msgs::LocationArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (mappr_msgs::Location lcmsg : msg->locations)
  {
    auto lc = locations_.find(lcmsg.label);
    if (lc != locations_.end())
    {
      lc->second->setMessage(lcmsg);
      lc->second->setShowLabel(showLabels_->getBool());
      lc->second->setCharacterHeight(labelSize_->getFloat());
    }
    else
    {
      ROS_ERROR_STREAM("create");
      locations_[lcmsg.label] = std::make_unique<LocationVisual>(context_->getSceneManager(), scene_node_);
      locations_[lcmsg.label]->setColor(boyton[curColor_++ % 9]);
      locations_[lcmsg.label]->setMessage(lcmsg);
      locations_[lcmsg.label]->setCharacterHeight(labelSize_->getFloat());
    }
  }

  curColor_ = curColor_ % 9;
}

void LocationArrayDisplay::slotLabelSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : locations_)
  {
    kv.second->setCharacterHeight(labelSize_->getFloat());
  }
}

void LocationArrayDisplay::slotShowLabels()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : locations_)
  {
    kv.second->setShowLabel(showLabels_->getBool());
  }
}

}  // namespace viz
}  // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::LocationArrayDisplay, rviz::Display)