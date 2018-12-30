#pragma once

#include "mappr_msgs/Viewpoint.h"

#include <QColor>

namespace Ogre
{
class Vector3;
class Quaternion;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace rviz
{
class MovableText;
}

namespace mappr
{
namespace viz
{
class Viewpoint
{
public:
  Viewpoint(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~Viewpoint();

  // Configure the visual to show the data in the message.
  void setMessage(const mappr_msgs::Viewpoint msg);

private:
  Ogre::SceneNode* text_node_{ nullptr };
  Ogre::SceneManager* scene_manager_{ nullptr };
  rviz::MovableText* text_{ nullptr };

  bool showLabel_;
};
}  // namespace viz
}  // namespace mappr