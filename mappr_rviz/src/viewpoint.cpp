#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <QColor>

#include <ros/ros.h>

#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/properties/parse_color.h>

#include "mappr_rviz/viewpoint.h"

namespace mappr
{
namespace viz
{
Viewpoint::Viewpoint(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Todo TF/HEADER cntxt
  text_node_ = parent_node->createChildSceneNode();
}

Viewpoint::~Viewpoint()
{
  scene_manager_->destroySceneNode(text_node_);
}

// ----------------------------------------------------------------------------
void Viewpoint::setMessage(const mappr_msgs::Viewpoint msg)
{
  if (text_ == nullptr)
  {
    text_ = new rviz::MovableText(msg.label);
    text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    text_node_->attachObject(text_);
  }

  auto center = msg.pose;
  Ogre::Vector3 pos = Ogre::Vector3(center.x, center.y, 0);
  Ogre::Vector3 scale = Ogre::Vector3(1, 1, 1);
  Ogre::Quaternion orient = Ogre::Quaternion(1, 0, 0, 0);
  text_node_->setPosition(pos);

  text_->setCharacterHeight(1);
  // text_->setColor(rviz::qtToOgre(color_));
  text_->setCaption(msg.label);
}

}  // namespace viz
}  // namespace mappr