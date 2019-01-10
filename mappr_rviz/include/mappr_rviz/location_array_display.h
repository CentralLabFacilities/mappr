#pragma once

#ifndef Q_MOC_RUN
#include "mappr_msgs/LocationArray.h"
#include "mappr_rviz/location_visual.h"
#include "mappr_rviz/topic_display.h"
#include "rviz/message_filter_display.h"
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
class LocationArrayDisplay : public TopicDisplay<mappr_msgs::LocationArray>
{
  Q_OBJECT
public:
  LocationArrayDisplay();
  ~LocationArrayDisplay() override = default;

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void slotShowLabels();
  void slotLabelSize();

private:
  void processMessage(const mappr_msgs::LocationArray::ConstPtr& msg) override;

  mappr_msgs::LocationArray::ConstPtr initMsg_;
  rviz::BoolProperty* showLabels_;
  rviz::FloatProperty* labelSize_;

  std::mutex mutex_;
  std::map<std::string, std::unique_ptr<LocationVisual> > locations_;

  int curColor_{ 0 };
};
}  // namespace viz
}  // namespace mappr