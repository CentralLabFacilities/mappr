#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreQuaternion.h>

#include <QColor>

#include <ros/ros.h>

#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/properties/parse_color.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>

#include "mappr_rviz/viewpoint_visual.h"

namespace mappr
{
namespace viz
{
ViewpointVisual::ViewpointVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Todo TF/HEADER cntxt
  pose_node_ = parent_node->createChildSceneNode();
  object_node_ = pose_node_->createChildSceneNode();
  text_node_ = pose_node_->createChildSceneNode();

  if (rviz::loadMeshFromResource(viewpointRes_).isNull())
  {
    ROS_ERROR("PlantFlagTool: failed to load model resource '%s'.", viewpointRes_.c_str());
    return;
  }

  Ogre::Entity* entity = scene_manager_->createEntity(viewpointRes_);
  object_node_->attachObject(entity);

  text_ = new rviz::MovableText("viewpoint");
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_node_->attachObject(text_);
}

ViewpointVisual::~ViewpointVisual()
{
  scene_manager_->destroySceneNode(object_node_);
}

void ViewpointVisual::setShowLabel(const bool show)
{
  text_node_->setVisible(show);
}

void ViewpointVisual::setCharacterHeight(const float size)
{
  text_size_ = size;
  if (text_ != nullptr)
  {
    text_->setCharacterHeight(text_size_);
    text_node_->setPosition(Ogre::Vector3(-1 * text_size_, 0, 1.5));
  }
}

// ----------------------------------------------------------------------------
void ViewpointVisual::setMessage(const mappr_msgs::Viewpoint msg)
{
  auto center = msg.pose;
  Ogre::Vector3 pos = Ogre::Vector3(center.x, center.y, 0);
  pose_node_->setPosition(pos);

  text_->setCaption(msg.label);

  float rot = center.theta;
  Ogre::Radian r = Ogre::Radian(rot);
  object_node_->setOrientation(Ogre::Quaternion(r, Ogre::Vector3::UNIT_Z));

  text_->setCharacterHeight(text_size_);
  text_node_->setPosition(Ogre::Vector3(-1 * text_size_, 0, 1.5));
  // text_->setColor(rviz::qtToOgre(color_));
}

}  // namespace viz
}  // namespace mappr