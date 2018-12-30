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

class InteractiveMarker : public QObject
{
  Q_OBJECT
public:
  InteractiveMarker(Ogre::SceneNode* scene_node, DisplayContext* context);
  ~InteractiveMarker() override;

  // reset contents to reflect the data from a new message
  // @return success
  bool processMessage(const visualization_msgs::InteractiveMarker& message);

  // reset contents to reflect the data from a new message
  // @return success
  void processMessage(const visualization_msgs::InteractiveMarkerPose& message);

  // called every frame update
  void update(float wall_dt);

  // directly set the pose, relative to parent frame
  // if publish is set to true, publish the change
  void setPose(Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string& control_name);

  void translate(Ogre::Vector3 delta_position, const std::string& control_name);
  void rotate(Ogre::Quaternion delta_orientation, const std::string& control_name);

  // schedule a pose reset once dragging is finished
  void requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation);

  void startDragging();
  void stopDragging();

  const Ogre::Vector3& getPosition()
  {
    return position_;
  }
  const Ogre::Quaternion& getOrientation()
  {
    return orientation_;
  }

  float getSize()
  {
    return scale_;
  }
  const std::string& getReferenceFrame()
  {
    return reference_frame_;
  }
  const std::string& getName()
  {
    return name_;
  }

  // show name above marker
  void setShowDescription(bool show);

  // show axes in origin
  void setShowAxes(bool show);

  // show visual helpers while dragging
  void setShowVisualAids(bool show);

  // @return true if the mouse event was intercepted, false if it was ignored
  bool handleMouseEvent(ViewportMouseEvent& event, const std::string& control_name);

  /**
   * Supports selection and menu events from a 3D cursor.
   *
   * @param  event        A struct holding certain event data (see full
   * description InteractiveMarkerControl::handle3DCursorEvent)
   * @param  cursor_pos   The world-relative position of the 3D cursor.
   * @param  cursor_rot   The world-relative orientation of the 3D cursor.
   * @param  control_name The name of the child InteractiveMarkerControl calling
   * this function.
   * @return              true if the cursor event was intercepted, false if it
   * was ignored
   */
  bool handle3DCursorEvent(ViewportMouseEvent& event, const Ogre::Vector3& cursor_pos,
                           const Ogre::Quaternion& cursor_rot, const std::string& control_name);

  /**
   * Pop up the context menu for this marker.
   *
   * @param  event         A struct holding certain event data (see full
   * description on InteractiveMarkerControl::handle3DCursorEvent)
   * @param  control_name  The name of the InteractiveMarkerControl that was
   * selected.
   * @param  three_d_point The world-relative position associated with this
   * mouse-click or cursor event.
   * @param  valid_point   True if three_d_point is valid (e.g. if the mouse ray
   * was successfully intersected with marker geometry).
   */
  void showMenu(ViewportMouseEvent& event, const std::string& control_name, const Ogre::Vector3& three_d_point,
                bool valid_point);

  // fill in current marker pose & name, publish
  void publishFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback, bool mouse_point_valid = false,
                       const Ogre::Vector3& mouse_point_rel_world = Ogre::Vector3(0, 0, 0));

  bool hasMenu()
  {
    return has_menu_;
  }

  /** @return A shared_ptr to the QMenu owned by this InteractiveMarker. */
  boost::shared_ptr<QMenu> getMenu()
  {
    return menu_;
  }

Q_SIGNALS:

  void userFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  void statusUpdate(StatusProperty::Level level, const std::string& name, const std::string& text);

protected Q_SLOTS:
  void handleMenuSelect(int menu_item_id);

protected:
  void publishPose();

  void reset();

  // set the pose of the parent frame, relative to the fixed frame
  void updateReferencePose();

  QString makeMenuString(const std::string& entry);

  // Recursively append menu and submenu entries to menu, based on a
  // vector of menu entry id numbers describing the menu entries at the
  // current level.
  void populateMenu(QMenu* menu, std::vector<uint32_t>& ids);

  DisplayContext* context_;

  // pose of parent coordinate frame
  std::string reference_frame_;
  ros::Time reference_time_;
  bool frame_locked_;

  // node representing reference frame in tf, like /map, /base_link, /head, etc.
  Ogre::SceneNode* reference_node_;

  // pose being controlled, relative to reference frame
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  // has the pose changed since the last feedback was sent?
  bool pose_changed_;
  double time_since_last_feedback_;

  using InteractiveMarkerControlPtr = boost::shared_ptr<InteractiveMarkerControl>;
  typedef std::map<std::string, InteractiveMarkerControlPtr> M_ControlPtr;
  M_ControlPtr controls_;

  std::string name_;
  std::string description_;

  bool dragging_;

  // pose being controlled
  bool pose_update_requested_;
  Ogre::Vector3 requested_position_;
  Ogre::Quaternion requested_orientation_;

  float scale_;

  boost::shared_ptr<QMenu> menu_;
  bool has_menu_;

  // Helper to more simply represent the menu tree.
  struct MenuNode
  {
    visualization_msgs::MenuEntry entry;
    std::vector<uint32_t> child_ids;
  };

  // maps menu index to menu entry and item
  std::map<uint32_t, MenuNode> menu_entries_;

  // Helper to store the top level of the menu tree.
  std::vector<uint32_t> top_level_menu_ids_;

  // which control has popped up the menu
  std::string last_control_name_;

  double heart_beat_t_;

  // visual aids

  Axes* axes_;

  InteractiveMarkerControlPtr description_control_;

  std::string topic_ns_;
  std::string client_id_;

  boost::recursive_mutex mutex_;

  boost::shared_ptr<boost::thread> sys_thread_;

  bool got_3d_point_for_menu_;
  Ogre::Vector3 three_d_point_for_menu_;

  bool show_visual_aids_;
};

}  // namespace rviz
