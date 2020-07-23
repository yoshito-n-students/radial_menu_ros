#ifndef RADIAL_MENU_BACKEND_BACKEND_NODELET_HPP
#define RADIAL_MENU_BACKEND_BACKEND_NODELET_HPP

#include <nodelet/nodelet.h>
#include <radial_menu_backend/backend_config.hpp>
#include <radial_menu_backend/backend_controller.hpp>
#include <radial_menu_backend/menu.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
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

    menu_ = Menu::fromParam(pnh.resolveName("menu"));
    if (!menu_) {
      throw ros::Exception("Cannot get a menu tree from the param '" + pnh.resolveName("menu") +
                           "'");
    }
    NODELET_INFO_STREAM("Menu:\n" << menu_->toString());

    controller_ = boost::make_shared< BackendController >(
        menu_, BackendConfig::fromParamNs(pnh.getNamespace()));

    state_pub_ = nh.advertise< radial_menu_msgs::State >("radial_menu_state", 1, true);
    state_pub_.publish(menu_->toState(ros::Time::now(), /* is_enabled = */ false));
    joy_sub_ = nh.subscribe("joy", 1, &BackendNodelet::onJoyRecieved, this);
  }

  void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
    // update menu state and publish
    state_pub_.publish(controller_->update(*joy));
    // NODELET_DEBUG_STREAM("Updated menu:\n" << menu_->toString());
  }

protected:
  MenuPtr menu_;
  BackendControllerPtr controller_;

  ros::Subscriber joy_sub_;
  ros::Publisher state_pub_;
};
} // namespace radial_menu_backend

#endif