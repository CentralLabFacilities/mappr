#include "mappr_rviz/viewpoint_array_display.h"

#include <QString>
#include <memory>

#include <rviz/properties/bool_property.h>

namespace mappr
{
namespace viz
{
ViewpointArrayDisplay::ViewpointArrayDisplay()
{
  showLabels_ = new rviz::BoolProperty("show Labels", true, "Draw the Labels.", this, SLOT(slotShowLabels()));

  /* TODO
  - show visual circle + dir + label
  - viewpoint creation tool

  - display interaction node
    - on drag emit signals
    - react on drag finished, update pose

  - show old + new pose (opague)
  - add rename menu

  */
}

void ViewpointArrayDisplay::onInitialize()
{
  initialized = false;
  firstMessage(nullptr);
}

void ViewpointArrayDisplay::reset()
{
  initialized = false;
}

void ViewpointArrayDisplay::firstMessage(const mappr_msgs::ViewpointArray::ConstPtr& /*msg*/)
{
  initialized = true;

  // test me here
}

void ViewpointArrayDisplay::processMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg)
{
  for (mappr_msgs::Viewpoint vpmsg : msg->viewpoints)
  {
    auto vp = viewpoints_.find(vpmsg.label);
    if (vp != viewpoints_.end())
    {
      vp->second->setMessage(vpmsg);
    }
    else
    {
      viewpoints_[vpmsg.label] = std::make_unique<Viewpoint>(context_->getSceneManager(), scene_node_);
      viewpoints_[vpmsg.label]->setMessage(vpmsg);
    }
  }

  if (!initialized)
  {
    firstMessage(msg);
  }
}

void ViewpointArrayDisplay::slotShowLabels()
{
  ROS_DEBUG_STREAM("show labels: " << ((showLabels_->getBool()) ? "true" : "false"));
}

}  // namespace viz
}  // namespace mappr
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mappr::viz::ViewpointArrayDisplay, rviz::Display)