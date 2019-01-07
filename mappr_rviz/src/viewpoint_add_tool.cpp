
#include <tf/transform_listener.h>

#include <mappr_msgs/Viewpoint.h>
#include <mappr_msgs/UpdateViewpoint.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "mappr_rviz/viewpoint_add_tool.h"

namespace mappr
{
namespace viz
{
ViewpointAddTool::ViewpointAddTool()
{
  shortcut_key_ = 'v';

  name_property_ = new rviz::StringProperty("Name", "viewpoint", "The name of the viewpoint.", getPropertyContainer(),
                                            SLOT(updateName()), this);
}

void ViewpointAddTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add Viewpoint");
  updateName();

  auto topic = "/mappr_server/add_viewpoint";
  client_ = nh_.serviceClient<mappr_msgs::UpdateViewpoint>(topic);
  ROS_INFO_STREAM("ViewpointAddTool connecting to " << topic);
}

void ViewpointAddTool::updateName()
{
  // NOP
}

void ViewpointAddTool::onPoseSet(double x, double y, double theta)
{
  mappr_msgs::Viewpoint viewpoint;

  viewpoint.header.frame_id = context_->getFixedFrame().toStdString();
  viewpoint.header.stamp = ros::Time::now();

  viewpoint.label = name_property_->getStdString();
  viewpoint.pose.x = x;
  viewpoint.pose.y = y;
  viewpoint.pose.theta = theta;

  mappr_msgs::UpdateViewpoint srv;
  srv.request.viewpoint = viewpoint;

  ROS_INFO_STREAM("add vp:\n" << viewpoint);
  client_.call(srv);
}

}  // namespace viz
}  // namespace mappr

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mappr::viz::ViewpointAddTool, rviz::Tool)