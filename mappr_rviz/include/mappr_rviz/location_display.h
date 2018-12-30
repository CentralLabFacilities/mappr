#pragma once

#ifndef Q_MOC_RUN
#include "mappr_msgs/Location.h"
#include "mappr_rviz/location_visual.h"
#include "mappr_rviz/topic_display.h"
#include "rviz/message_filter_display.h"
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
class LocationDisplay : public TopicDisplay<mappr_msgs::Location>
{
  Q_OBJECT
public:
  LocationDisplay();
  ~LocationDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void slotShowLabel();

private:
  void processMessage(const mappr_msgs::Location::ConstPtr& msg) override;

  bool initialized;
  void firstMessage(const mappr_msgs::Location::ConstPtr& msg);

  mappr_msgs::Location::ConstPtr initMsg_;
  rviz::BoolProperty* showLabel_;

  std::unique_ptr<LocationVisual> locationVis_;
};
}  // namespace viz
}  // namespace mappr