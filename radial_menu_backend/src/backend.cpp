#include <radial_menu_backend/menu.hpp>
#include <radial_menu_backend/menu_config.hpp>
#include <radial_menu_backend/menu_controller.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

namespace rmb = radial_menu_backend;

rmb::MenuPtr menu;
rmb::MenuControllerPtr controller;
ros::Publisher state_pub;

// callback
void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
  // update menu state and publish
  state_pub.publish(controller->update(*joy));
  ROS_INFO_STREAM("Updated menu:\n" << menu->toString());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "backend");
  ros::NodeHandle nh;

  // load menu tree
  menu = rmb::Menu::fromParam("~menu");
  if (!menu) {
    return 1;
  }

  // menu controller
  controller.reset(new rmb::MenuController(menu, rmb::MenuConfig::fromParamNs("~")));

  //
  state_pub = nh.advertise< radial_menu_msgs::State >("radial_menu_state", 1, true);
  ros::Subscriber joy_sub(nh.subscribe("joy", 1, onJoyRecieved));

  //
  ros::spin();

  return 0;
}