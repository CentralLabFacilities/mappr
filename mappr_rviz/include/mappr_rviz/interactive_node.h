#pragma once

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#endif

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerPose.h>

#include <ros/publisher.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/selection/forwards.h>

#include <rviz/default_plugin/interactive_markers/interactive_marker_control.h>
#include <rviz/properties/status_property.h>

namespace Ogre
{
class SceneNode;
}

class QMenu;

namespace rviz
{
class DisplayContext;
class InteractiveMarkerDisplay;
}  // namespace rviz

namespace mappr
{
namespace viz
{
/*
  drageable thing for vps and location edges
*/
class InteractiveNode : public QObject
{
  Q_OBJECT
public:
  InteractiveNode(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~InteractiveNode() override;

Q_SIGNALS:
  void userFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  void statusUpdate(rviz::StatusProperty::Level level, const std::string& name, const std::string& text);

private:
  Ogre::SceneManager* scene_manager_{ nullptr };
  Ogre::SceneNode* node_{ nullptr };
};

}  // namespace viz
}  // namespace mappr
