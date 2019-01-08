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
InteractiveNode::InteractiveNode(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  node_ = parent_node->createChildSceneNode();
  // axes_ = new Axes(context->getSceneManager(), reference_node_, 1, 0.05);
}

InteractiveNode::~InteractiveNode()
{
  // delete axes_;
  // context_->getSceneManager()->destroySceneNode(reference_node_);
}

}  // namespace viz
}  // namespace mappr