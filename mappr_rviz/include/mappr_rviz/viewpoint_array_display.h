#pragma once

#ifndef Q_MOC_RUN
#include <mappr_msgs/ViewpointArray.h>

#include "mappr_rviz/viewpoint.h"
#include "mappr_rviz/topic_display.h"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class BoolProperty;
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

private:
  void processMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg) override;

  bool initialized;
  void firstMessage(const mappr_msgs::ViewpointArray::ConstPtr& msg);

  mappr_msgs::ViewpointArray::ConstPtr initMsg_;
  rviz::BoolProperty* showLabels_;

  std::map<std::string, std::unique_ptr<Viewpoint> > viewpoints_;
};
}  // namespace viz
}  // namespace mappr