#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <QColor>

#include "ros/ros.h"

#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/properties/parse_color.h"

#include "mappr_rviz/location_visual.h"
#include "mappr_rviz/polygon_util.h"

namespace mappr
{
namespace viz
{
LocationVisual::LocationVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  location_node_ = parent_node->createChildSceneNode();
  text_node_ = parent_node->createChildSceneNode();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  location_node_->attachObject(manual_object_);

  color_ = QColor(25, 255, 0);
}

LocationVisual::~LocationVisual()
{
  scene_manager_->destroySceneNode(location_node_);
  scene_manager_->destroySceneNode(text_node_);
}

// ----------------------------------------------------------------------------
void LocationVisual::setMessage(const mappr_msgs::Location::ConstPtr& msg)
{
  // Ogre::Vector3 position;
  // Ogre::Quaternion orientation;
  // if( !context_->getFrameManager()->getTransform( msg->header, position,
  // orientation ))
  // {
  //   ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
  //              msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  // }

  // location_node_->setPosition( position );
  // location_node_->setOrientation( orientation );

  manual_object_->clear();

  // TODO(lruegeme): check if msg changed

  Ogre::ColourValue color = rviz::qtToOgre(color_);
  color.a = 1;

  uint32_t num_points = msg->polygon.size();
  if (num_points > 0)
  {
    manual_object_->estimateVertexCount(num_points);
    manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    for (uint32_t i = 0; i < num_points + 1; ++i)
    {
      const geometry_msgs::Point& msg_point = msg->polygon[i % num_points];
      manual_object_->position(msg_point.x, msg_point.y, 0);
      manual_object_->colour(color);
    }

    manual_object_->end();
  }

  if (text_ == nullptr)
  {
    text_ = new rviz::MovableText(msg->label);
    text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    text_node_->attachObject(text_);
  }

  auto center = compute2DPolygonCentroid(msg->polygon);
  Ogre::Vector3 pos = Ogre::Vector3(center.x, center.y, 0);
  Ogre::Vector3 scale = Ogre::Vector3(1, 1, 1);
  Ogre::Quaternion orient = Ogre::Quaternion(1, 0, 0, 0);
  text_node_->setPosition(pos);

  text_->setCharacterHeight(1);
  text_->setColor(rviz::qtToOgre(color_));
  text_->setCaption(msg->label);
}

}  // namespace viz
}  // namespace mappr