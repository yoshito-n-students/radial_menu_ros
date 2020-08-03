#ifndef RADIAL_MENU_BACKEND_BACKEND_NODELET_HPP
#define RADIAL_MENU_BACKEND_BACKEND_NODELET_HPP

#include <nodelet/nodelet.h>
#include <radial_menu_backend/backend_config.hpp>
#include <radial_menu_backend/backend_controller.hpp>
#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

#include <boost/make_shared.hpp>

namespace radial_menu_backend {

class BackendNodelet : public nodelet::Nodelet {
public:
  BackendNodelet() {}

  virtual ~BackendNodelet() {}

protected:
  virtual void onInit() {
    ros::NodeHandle nh(getNodeHandle()), pnh(getPrivateNodeHandle());

    model_.reset(new radial_menu_model::Model());
    if (!model_->setDescriptionFromParam(nh.resolveName("menu_description"))) {
      throw ros::Exception("Cannot set a model description from the param '" +
                           nh.resolveName("menu_description") + "'");
    }
    NODELET_INFO_STREAM("Menu:\n" << model_->toString());

    controller_ = boost::make_shared< BackendController >(
        model_, BackendConfig::fromParamNs(pnh.getNamespace()));

    state_pub_ = nh.advertise< radial_menu_msgs::State >("menu_state", 1, true);
    state_pub_.publish(model_->exportState());
    joy_sub_ = nh.subscribe("joy", 1, &BackendNodelet::onJoyRecieved, this);
  }

  void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
    // update menu state and publish
    state_pub_.publish(controller_->update(*joy));
    // NODELET_DEBUG_STREAM("Updated menu:\n" << menu_->toString());
  }

protected:
  radial_menu_model::ModelPtr model_;
  BackendControllerPtr controller_;

  ros::Subscriber joy_sub_;
  ros::Publisher state_pub_;
};
} // namespace radial_menu_backend

#endif