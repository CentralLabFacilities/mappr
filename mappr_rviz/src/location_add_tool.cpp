#include "mappr_rviz/location_add_tool.h"

#include <tf/transform_listener.h>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <rviz/geometry.h>
#include <rviz/render_panel.h>
#include <rviz/properties/parse_color.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

namespace mappr
{
namespace viz
{
LocationAddTool::LocationAddTool() : Tool()
{
}
LocationAddTool::~LocationAddTool()
{
  scene_manager_->destroySceneNode(lines_node_);
  // delete lines_;
}

void LocationAddTool::onInitialize()
{
  auto parent_node = scene_manager_->getRootSceneNode();
  lines_node_ = parent_node->createChildSceneNode();
  lines_node_->setVisible(false);

  lines_ = scene_manager_->createManualObject();
  lines_->setDynamic(true);
  lines_node_->attachObject(lines_);
}

void LocationAddTool::activate()
{
  setStatus("Click to add start point.");
  state_ = First;
}

void LocationAddTool::deactivate()
{
  clicked_points_.clear();
  state_ = First;
  lines_node_->setVisible(false);
  lines_->clear();
}

int LocationAddTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection))
  {
    setStatus("NO INTERSECTION!");
  }

  setCurrentPoint(intersection);

  if (event.leftDown())
  {
    if(state_ == Add) {
      //heckIFClickedFirstPoint
      auto dist = intersection.distance(clicked_points_.front());
      std::cout << dist << std::endl;
      if(dist < 0.2) {
        finish();
        return Finished;
      }
    }

    addPoint(intersection);
  }
  else if (event.rightDown())
  {
    deletePoint();
  }
}

void LocationAddTool::onFinish(std::list<Ogre::Vector3> points)
{
  // send new location
  std::cout << "finished" << std::endl;
}

void LocationAddTool::finish() {
  onFinish(clicked_points_);
  deactivate();
}

void LocationAddTool::addPoint(Ogre::Vector3 point)
{
  clicked_points_.push_back(point);
  state_ = Add;
}

void LocationAddTool::deletePoint()
{
  clicked_points_.pop_back();

  if (clicked_points_.empty())
  {
    state_ = First;
  }
}

void LocationAddTool::setCurrentPoint(Ogre::Vector3 cur_point)
{
  if (state_ == First)
  {
    lines_->clear();
    lines_node_->setVisible(false);
    return;
  }

  lines_node_->setVisible(true);
  lines_->clear();

  Ogre::ColourValue color = rviz::qtToOgre(color_);
  color.a = 1;

  uint32_t num_points = clicked_points_.size();
  if (num_points > 0)
  {
    lines_->estimateVertexCount(num_points + 2);

    lines_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN );
    lines_->position(clicked_points_.front().x, clicked_points_.front().y, 0);
    lines_->colour(rviz::qtToOgre( QColor(255, 0, 0)));
    lines_->position(clicked_points_.front().x - 0.1, clicked_points_.front().y , 0);
    lines_->colour(rviz::qtToOgre( QColor(255, 0, 0)));
    lines_->position(clicked_points_.front().x - 0.1, clicked_points_.front().y - 0.1, 0);
    lines_->colour(rviz::qtToOgre( QColor(255, 0, 0)));

    lines_->end();

    lines_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (auto p : clicked_points_)
    {
      lines_->position(p.x, p.y, 0);
      lines_->colour(color);
    }

    lines_->position(cur_point.x, cur_point.y, 0);
    lines_->colour(color);

    lines_->end();
  }
}

void LocationAddTool::updateName()
{
  // Only lowercase names
  name_property_->setString(name_property_->getString().toLower());
}

}  // namespace viz
}  // namespace mappr

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mappr::viz::LocationAddTool, rviz::Tool)