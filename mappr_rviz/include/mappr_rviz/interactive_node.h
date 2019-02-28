#pragma once

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>

#include <OgreRay.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreSceneManager.h>
#endif

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <geometry_msgs/Pose2D.h>

#include <ros/publisher.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/selection/forwards.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/interactive_object.h>

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
}  // namespace rviz

namespace mappr
{
namespace viz
{
/*
  drageable thing for vps and location edges
*/
class InteractiveNode : public Ogre::SceneManager::Listener, public rviz::InteractiveObject
{
public:
  InteractiveNode(Ogre::SceneManager* scene_manager /*, DisplayContext* context*/);
  ~InteractiveNode() override;

  void initialize();
  bool updatePose(geometry_msgs::Pose2D pose);
  void setEnabled(bool enabled);

  // rviz::InteractiveObject
  virtual bool isInteractive();
  virtual void enableInteraction(bool enable);
  virtual void handleMouseEvent(rviz::ViewportMouseEvent& event);
  virtual const QCursor& getCursor() const;

  // ??
  virtual void handle3DCursorEvent(rviz::ViewportMouseEvent event, const Ogre::Vector3& cursor_3D_pos,
                                   const Ogre::Quaternion& cursor_3D_orientation);

  // Ogre::SceneManager::Listener
  virtual void preFindVisibleObjects(Ogre::SceneManager* source, Ogre::SceneManager::IlluminationRenderStage irs,
                                     Ogre::Viewport* v);

Q_SIGNALS:
  void userFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  // void statusUpdate(rviz::StatusProperty::Level level, const std::string& name, const std::string& text);

private:
  Ogre::SceneManager* scene_manager_{ nullptr };
  rviz::DisplayContext* context_{ nullptr };
  Ogre::SceneNode* node_{ nullptr };

  bool dragging_{ false };
  bool enabled_{ false };

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};

}  // namespace viz
}  // namespace mappr
