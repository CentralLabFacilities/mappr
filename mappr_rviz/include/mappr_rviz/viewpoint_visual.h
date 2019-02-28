#pragma once

#include "mappr_msgs/Viewpoint.h"

#include <QColor>

namespace Ogre
{
class Entity;
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
class ViewpointVisual
{
public:
  ViewpointVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, mappr_msgs::Viewpoint msg,
                  bool show = true, float char_heigth = 0.5);
  ~ViewpointVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const mappr_msgs::Viewpoint msg);
  void setShowLabel(const bool show);
  void setCharacterHeight(const float size);

private:
  Ogre::SceneNode* text_node_{ nullptr };
  Ogre::SceneNode* object_node_{ nullptr };
  Ogre::SceneNode* pose_node_{ nullptr };

  Ogre::SceneManager* scene_manager_{ nullptr };
  rviz::MovableText* text_{ nullptr };

  float text_size_;
  std::string viewpointRes_{ "package://mappr_rviz/media/viewpoint.dae" };
};
}  // namespace viz
}  // namespace mappr