#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
}

namespace mappr
{
namespace viz
{
class ViewpointAddTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  ViewpointAddTool();
  virtual ~ViewpointAddTool()
  {
  }
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateName();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  rviz::StringProperty* name_property_;
};

}  // viz
}  // mappr
