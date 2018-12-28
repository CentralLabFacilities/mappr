#pragma once

#include "mappr_msgs/Location.h"

#include <QColor>

namespace Ogre {
class Vector3;
class Quaternion;
class ManualObject;
class SceneManager;
class SceneNode;
} // namespace Ogre

namespace rviz {
class MovableText;
}

namespace mappr {
namespace viz {
class LocationVisual {
public:
  LocationVisual(Ogre::SceneManager *scene_manager,
                 Ogre::SceneNode *parent_node);
  ~LocationVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const mappr_msgs::Location::ConstPtr &msg);

private:
  Ogre::SceneNode *location_node_;
  Ogre::SceneNode *text_node_;
  Ogre::SceneManager *scene_manager_;

  Ogre::ManualObject *manual_object_;
  rviz::MovableText *text_;

  bool showLabel_;
  QColor color_;
};
} // namespace viz
} // namespace mappr