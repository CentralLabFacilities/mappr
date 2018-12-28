#pragma once

#ifndef Q_MOC_RUN
#include "mappr_msgs/LocationArray.h"
#include "mappr_rviz/location_visual.h"
#include "rviz/message_filter_display.h"
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class BoolProperty;
}

namespace mappr {
namespace viz {
class LocationArrayDisplay : public rviz::Display {
  Q_OBJECT
public:
  LocationArrayDisplay();
  virtual ~LocationArrayDisplay() = default;

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void slotShowLabels();

private:
  void processMessage(const mappr_msgs::LocationArray::ConstPtr &msg);

  bool initialized;
  void firstMessage(const mappr_msgs::LocationArray::ConstPtr &msg);

  mappr_msgs::LocationArray::ConstPtr initMsg_;
  rviz::BoolProperty *showLabels_;

  std::unique_ptr<LocationVisual> locationVis_;
};
} // namespace viz
} // namespace mappr