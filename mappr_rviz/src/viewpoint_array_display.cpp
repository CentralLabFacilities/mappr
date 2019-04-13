#include "mappr_rviz/viewpoint_array_display.h"

#include <QString>
#include <memory>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

#include "mappr_rviz/interactive_node.h"

namespace mappr
{
namespace viz
{
ViewpointArrayDisplay::ViewpointArrayDisplay()
{
  showLabels_ = new rviz::BoolProperty("show Labels", true, "Draw the Labels.", this, SLOT(slotShowLabels()));
  showParent_ = new rviz::BoolProperty("show Parent Location", false, "Prefix label with Parent Location", this, SLOT(slotShowParent()));
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
  //InteractiveNode* inode = new InteractiveNode(context_->getSceneManager());
}

void ViewpointArrayDisplay::reset()
{
  TDClass::reset();
}

void ViewpointArrayDisplay::processMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : viewpoints_)
  {
    ROS_DEBUG_STREAM(kv.first);
  }

  for (mappr_msgs::Viewpoint vpmsg : msg->viewpoints)
  {
    auto uuid = vpmsg.label + vpmsg.parent_location_name;
    auto vp = viewpoints_.find(uuid);
    if (vp != viewpoints_.end())
    {
      vp->second->setCharacterHeight(labelSize_->getFloat());
      vp->second->setShowLabel(showLabels_->getBool());
      vp->second->setMessage(vpmsg);
    }
    else
    {
      viewpoints_[uuid] = std::make_unique<ViewpointVisual>(context_->getSceneManager(), scene_node_, vpmsg,
                                                                   showLabels_->getBool(), showParent_->getBool(), labelSize_->getFloat());
    }
  }
}

void ViewpointArrayDisplay::slotLabelSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : viewpoints_)
  {
    kv.second->setCharacterHeight(labelSize_->getFloat());
  }
}

void ViewpointArrayDisplay::slotShowLabels()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : viewpoints_)
  {
    kv.second->setShowLabel(showLabels_->getBool());
  }
}

void ViewpointArrayDisplay::slotShowParent()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& kv : viewpoints_)
  {
    kv.second->setShowParent(showParent_->getBool());
  }
}

}  // namespace viz
}  // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::ViewpointArrayDisplay, rviz::Display)