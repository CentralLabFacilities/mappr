#pragma once

/**
 * MessageFilterDisplay without TF (msgs without header)
 */

#ifndef Q_MOC_RUN
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <message_filters/subscriber.h>
#endif

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/ros_topic_property.h"

#include "rviz/display.h"

namespace mappr
{
namespace viz
{
/** @brief Helper superclass for MessageFilterDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class _RosTopicDisplay : public rviz::Display
{
  Q_OBJECT
public:
  _RosTopicDisplay()
  {
    topic_property_ = new rviz::RosTopicProperty("Topic", "", "", "", this, SLOT(updateTopic()));
    unreliable_property_ =
        new rviz::BoolProperty("Unreliable", false, "Prefer UDP topic transport", this, SLOT(updateTopic()));
  }

protected Q_SLOTS:
  virtual void updateTopic() = 0;

protected:
  rviz::RosTopicProperty* topic_property_;
  rviz::BoolProperty* unreliable_property_;
};

/** @brief Display subclass using a tf2_ros::MessageFilter, templated on the ROS
 * message type.
 *
 * This class brings together some common things used in many Display
 * types.  It has a tf2_ros::MessageFilter to filter incoming messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  It also has an Ogre::SceneNode which  */
template <class MessageType>
class TopicDisplay : public _RosTopicDisplay
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.
public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  using TDClass = TopicDisplay<MessageType>;

  TopicDisplay()
  {
    QString message_type = QString::fromStdString(ros::message_traits::datatype<MessageType>());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  void onInitialize() override
  {
  }

  ~TopicDisplay() override
  {
    unsubscribe();
  }

  void reset() override
  {
    rviz::Display::reset();
    messages_received_ = 0;
  }

  void setTopic(const QString& topic, const QString& datatype) override
  {
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void subscribe()
  {
    if (!isEnabled())
    {
      return;
    }

    try
    {
      ros::TransportHints transport_hint = ros::TransportHints().reliable();
      // Determine UDP vs TCP transport for user selection.
      if (unreliable_property_->getBool())
      {
        transport_hint = ros::TransportHints().unreliable();
      }
      sub_.subscribe(update_nh_, topic_property_->getTopicStd(), 10, transport_hint);
      sub_.registerCallback(boost::bind(&TopicDisplay<MessageType>::incomingMessage, this, _1));
      setStatus(rviz::StatusProperty::Warn, "Topic", "no msgs");
    }
    catch (ros::Exception& e)
    {
      setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    sub_.unsubscribe();
  }

  void onEnable() override
  {
    subscribe();
  }

  void onDisable() override
  {
    unsubscribe();
    reset();
  }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(const typename MessageType::ConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }

    ++messages_received_;
    setStatus(rviz::StatusProperty::Ok, "Topic", QString::number(messages_received_) + " messages received");

    processMessage(msg);
  }

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(const typename MessageType::ConstPtr& msg) = 0;

  message_filters::Subscriber<MessageType> sub_;
  uint32_t messages_received_{ 0 };
};

}  // namespace viz
}  // namespace mappr