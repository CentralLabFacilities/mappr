#include <boost/make_shared.hpp>

#include <QMenu>

#include <OgreMaterialManager.h>
#include <OgreMath.h>
#include <OgreRenderWindow.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>

#include <interactive_markers/tools.h>
#include <ros/ros.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/geometry.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_quaternions.h>

#include <rviz/default_plugin/interactive_markers/integer_action.h>

#include "mappr_rviz/interactive_node.h"

namespace mappr
{
namespace viz
{
InteractiveNode::InteractiveNode(Ogre::SceneManager* scene_manager /*, DisplayContext* context*/)
{
  scene_manager_ = scene_manager;
  // context_ = context;
  node_ = scene_manager->createSceneNode();

  // Ogre::SceneManager::Listener
  scene_manager->addListener(this);

  // axes_ = new Axes(context->getSceneManager(), reference_node_, 1, 0.05);
  initialize();
}

void InteractiveNode::initialize()
{
  ROS_INFO_STREAM("inode init");
}

bool InteractiveNode::isInteractive()
{
  ROS_INFO_STREAM("isInteractibe");
}
void InteractiveNode::enableInteraction(bool enable)
{
  ROS_INFO_STREAM("enableInteraction");
}
void InteractiveNode::handleMouseEvent(rviz::ViewportMouseEvent& event)
{
  ROS_INFO_STREAM("handleMouseEvent");
}
const QCursor& InteractiveNode::getCursor() const
{
  ROS_INFO_STREAM("getCursor");
}

bool InteractiveNode::updatePose(geometry_msgs::Pose2D pose)
{
  if (dragging_)
    return false;

  position_ = Ogre::Vector3(pose.x, pose.y, 0);
  Ogre::Radian r = Ogre::Radian(pose.theta);
  orientation_ = Ogre::Quaternion(r, Ogre::Vector3::UNIT_Z);

  // context_->queueRender();
}

void InteractiveNode::setEnabled(bool enabled)
{
  enabled_ = enabled;
  node_->setVisible(enabled_);
  // context_->queueRender();
}

void InteractiveNode::handle3DCursorEvent(rviz::ViewportMouseEvent event, const Ogre::Vector3& cursor_3D_pos,
                                          const Ogre::Quaternion& cursor_3D_orientation)
{
  ROS_INFO_STREAM("handle3DCursorEvent");
}

// This is an Ogre::SceneManager::Listener function
void InteractiveNode::preFindVisibleObjects(Ogre::SceneManager* source, Ogre::SceneManager::IlluminationRenderStage irs,
                                            Ogre::Viewport* v)
{
  ROS_INFO_STREAM("PRE FIND VISO");
}

InteractiveNode::~InteractiveNode()
{
  // delete axes_;
  // context_->getSceneManager()->destroySceneNode(reference_node_);
}

}  // namespace viz
}  // namespace mappr