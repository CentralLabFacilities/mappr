#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#include "rviz/tool.h"
#endif

#include <QColor>

namespace rviz
{
class DisplayContext;
class StringProperty;
}  // namespace rviz

namespace Ogre
{
class Vector3;
class Quaternion;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace mappr
{
namespace viz
{
class LocationAddTool : public rviz::Tool
{
  Q_OBJECT
public:
  LocationAddTool();
  virtual ~LocationAddTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

protected:
  virtual void onFinish(std::list<Ogre::Vector3> points);

  void addPoint(Ogre::Vector3 point);
  void deletePoint();
  void setCurrentPoint(Ogre::Vector3 point);
  void finish();

  enum State
  {
    First,
    Add
  };
  State state_;

private Q_SLOTS:
  void updateName();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;

  rviz::StringProperty* name_property_;

  Ogre::SceneNode* lines_node_{ nullptr };
  std::list<Ogre::Vector3> clicked_points_;

  Ogre::ManualObject* lines_{ nullptr };

  QColor color_{ QColor(0, 255, 0) };
};

}  // viz
}  // mappr
