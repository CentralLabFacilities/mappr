#include "mappr_rviz/viewpoint_array_display.h"

#include <QString>
#include <memory>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

namespace mappr
{
namespace viz
{
ViewpointArrayDisplay::ViewpointArrayDisplay()
{
  showLabels_ = new rviz::BoolProperty("show Labels", true, "Draw the Labels.", this, SLOT(slotShowLabels()));
  labelSize_ = new rviz::FloatProperty("Label size", 0.5, "Character Height of TextLabel", this, SLOT(slotLabelSize()));

  /* TODO
  - remove deleted vps and locations
  - combine to arena display

  - display interaction node
    - on drag emit signals
    - react on drag finished, update pose

  - show old + new pose (opague)
  - add rename menu

  */
}

void ViewpointArrayDisplay::onInitialize()
{
  TDClass::onInitialize();
}

void ViewpointArrayDisplay::reset()
{
  TDClass::reset();
}

void ViewpointArrayDisplay::processMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg)
{
  for (mappr_msgs::Viewpoint vpmsg : msg->viewpoints)
  {
    auto vp = viewpoints_.find(vpmsg.label);
    if (vp != viewpoints_.end())
    {
      vp->second->setMessage(vpmsg);
      vp->second->setShowLabel(showLabels_->getBool());
      vp->second->setCharacterHeight(labelSize_->getFloat());
    }
    else
    {
      viewpoints_[vpmsg.label] = std::make_unique<ViewpointVisual>(context_->getSceneManager(), scene_node_);
      viewpoints_[vpmsg.label]->setMessage(vpmsg);
    }
  }
}

void ViewpointArrayDisplay::slotLabelSize()
{
  for (const auto& kv : viewpoints_)
  {
    kv.second->setCharacterHeight(labelSize_->getFloat());
  }
}

void ViewpointArrayDisplay::slotShowLabels()
{
  for (const auto& kv : viewpoints_)
  {
    kv.second->setShowLabel(showLabels_->getBool());
  }
}

}  // namespace viz
}  // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::ViewpointArrayDisplay, rviz::Display)