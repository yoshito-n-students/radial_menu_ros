#ifndef RADIAL_MENU_RVIZ_DISPLAY_BASE_HPP
#define RADIAL_MENU_RVIZ_DISPLAY_BASE_HPP

#include <memory>

#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/subscriber.h>
#include <rviz/display.h>

namespace radial_menu_rviz {

// base implementation of display classes except Qt's signals and slots
// because template classes cannot have any slots and signals by Qt's limitation
template < typename DrawingProperty, class PropertyControl, class ImageDrawer >
class DisplayBase : public rviz::Display {
public:
  DisplayBase() {}

  virtual ~DisplayBase() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    // allocate objects
    prop_ctl_.reset(new PropertyControl(this));
    model_.reset(new radial_menu_model::Model());
    drawer_.reset(new ImageDrawer(model_, prop_ctl_->drawingProperty()));
    overlay_.reset(new ImageOverlay());

    // apply the initial properties
    // (except subscription. it will be executed in onEnable().)
    updateDescription(prop_ctl_->descriptionProperty());
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

  void updateDescription(const DescriptionProperty &prop) {
    if (model_->setDescriptionFromParam(prop.param_name.toStdString())) {
      state_ = model_->exportState();
      updateImage();
    }
  }

  void updateSubscription(const SubscriptionProperty &prop) {
    // unsubscribe
    state_sub_.shutdown();

    // destroy the last state from the previous session
    model_->resetState();
    state_ = model_->exportState();
    updateImage();

    // subscribe the new topic
    try {
      state_sub_ =
          ros::NodeHandle().subscribe(prop.topic.toStdString(), 1, &DisplayBase::updateImage, this);
    } catch (const ros::Exception &error) {
      ROS_ERROR_STREAM(getName().toStdString()
                       << ": error on subscribing topic ('" << prop.topic.toStdString()
                       << "'): " << error.what());
    }
  }

  // update menu image based on the current configs
  void updateImage() {
    overlay_->setImage(drawer_->draw());
    overlay_->update();
  }

  // update menu image with the given menu state
  void updateImage(const radial_menu_msgs::StateConstPtr &new_state) {
    if (state_->is_enabled != new_state->is_enabled ||
        state_->pointed_id != new_state->pointed_id ||
        state_->selected_ids != new_state->selected_ids) {
      model_->setState(*new_state);
      state_ = new_state;
      updateImage();
    }
  }

  // update menu image with the given drawing property
  void updateImage(const DrawingProperty &prop) {
    drawer_->setProperty(prop);
    updateImage();
  }

  void updatePosition(const PositionProperty &prop) {
    overlay_->setOrigin(prop.origin);
    overlay_->update();
  }

protected:
  // property control via Rviz
  std::unique_ptr< PropertyControl > prop_ctl_;
  // menu tree model
  radial_menu_model::ModelPtr model_;
  // menu state subscriber
  ros::Subscriber state_sub_;
  radial_menu_msgs::StateConstPtr state_;
  // state drawer
  std::unique_ptr< ImageDrawer > drawer_;
  // overlay on Rviz
  std::unique_ptr< ImageOverlay > overlay_;
};
} // namespace radial_menu_rviz

#endif