#pragma once

#ifndef Q_MOC_RUN
#include <mappr_msgs/ViewpointArray.h>

#include "mappr_rviz/viewpoint_visual.h"
#include "mappr_rviz/topic_display.h"
#endif

#include <mutex>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class BoolProperty;
class FloatProperty;
}

namespace mappr
{
namespace viz
{
class ViewpointArrayDisplay : public TopicDisplay<mappr_msgs::ViewpointArray>
{
  Q_OBJECT
public:
  ViewpointArrayDisplay();
  ~ViewpointArrayDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void slotShowLabels();
  void slotLabelSize();

private:
  void processMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg) override;

  mappr_msgs::ViewpointArray::ConstPtr initMsg_;
  rviz::BoolProperty* showLabels_;
  rviz::FloatProperty* labelSize_;

  std::mutex mutex_;
  std::map<std::string, std::unique_ptr<ViewpointVisual> > viewpoints_;
};
}  // namespace viz
}  // namespace mappr