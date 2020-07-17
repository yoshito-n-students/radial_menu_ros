#ifndef RADIAL_MENU_RVIZ_DISPLAY_HPP
#define RADIAL_MENU_RVIZ_DISPLAY_HPP

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_drawer.hpp>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <radial_menu_rviz/property_control.hpp>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/subscriber.h>
#include <rviz/display.h>

#include <QObject>

#include <boost/scoped_ptr.hpp>

namespace radial_menu_rviz {

class Display : public rviz::Display {
  Q_OBJECT

public:
  Display() {}

  virtual ~Display() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    namespace rmm = radial_menu_msgs;

    // property control on the Display panel
    prop_ctl_.reset(new PropertyControl(this));

    // menu image generator
    drawer_.reset(new ImageDrawer(closedState(), prop_ctl_->drawingProperty()));

    // overlay on the main view
    overlay_.reset(new ImageOverlay());

    // slots on properties changed
    connect(prop_ctl_.get(), SIGNAL(subscriptionPropertyChanged(const SubscriptionProperty &)),
            this, SLOT(updateSubscription(const SubscriptionProperty &)));
    connect(prop_ctl_.get(), SIGNAL(drawingPropertyChanged(const DrawingProperty &)), this,
            SLOT(updateImage(const DrawingProperty &)));
    connect(prop_ctl_.get(), SIGNAL(positionPropertyChanged(const PositionProperty &)), this,
            SLOT(updatePosition(const PositionProperty &)));

    // manually execute the slots to apply the initial properties
    // (except subscription. it will be executed in onEnable().)
    updateImage(prop_ctl_->drawingProperty());
    updatePosition(prop_ctl_->positionProperty());
  }

  // called when enabled
  virtual void onEnable() {
    updateSubscription(prop_ctl_->subscriptionProperty());
    overlay_->show();
  }

  // called when disabled
  virtual void onDisable() {
    overlay_->hide();
    state_sub_.shutdown();
  }

  void updateState(const radial_menu_msgs::StateConstPtr &state) {
    drawer_->setState(*state);
    updateImage(prop_ctl_->drawingProperty());
  }

protected Q_SLOTS:
  void updateSubscription(const SubscriptionProperty &prop) {
    // unsubscribe
    state_sub_.shutdown();

    // destroy the last state from the previous session
    drawer_->setState(closedState());
    updateImage(prop_ctl_->drawingProperty());

    // subscribe the new topic
    try {
      state_sub_ =
          ros::NodeHandle().subscribe(prop.topic.toStdString(), 1, &Display::updateState, this);
    } catch (const ros::Exception &error) {
      ROS_ERROR_STREAM(getName().toStdString()
                       << ": error on subscribing topic ('" << prop.topic.toStdString()
                       << "'): " << error.what());
    }
  }

  void updateImage(const DrawingProperty &prop) {
    drawer_->setProperty(prop);
    overlay_->setImage(drawer_->draw());
  }

  void updatePosition(const PositionProperty &prop) { overlay_->setTopLeft(prop.top_left); }

protected:
  static radial_menu_msgs::State closedState() {
    radial_menu_msgs::State state;
    state.state = radial_menu_msgs::State::STATE_CLOSED;
    state.pointed_id = -1;
    return state;
  }

protected:
  // property control via Rviz
  boost::scoped_ptr< PropertyControl > prop_ctl_;

  // menu state subscriber
  ros::Subscriber state_sub_;

  // state drawer
  boost::scoped_ptr< ImageDrawer > drawer_;

  // overlay on Rviz
  boost::scoped_ptr< ImageOverlay > overlay_;
};
} // namespace radial_menu_rviz

#endif