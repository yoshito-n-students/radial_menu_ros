#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>

radial_menu_model::Model model;
std::string mode = "None";
bool paused = true;

void onSynchronized(const sensor_msgs::JoyConstPtr &joy,
                    const radial_menu_msgs::StateConstPtr &state) {
  namespace rmm = radial_menu_msgs;

  if (!model.setState(*state)) {
    ROS_ERROR("onSynchronized(): Cannot set state to the model");
    return;
  }

  // parse the menu state
  std::string new_mode;
  bool new_paused;
  if (model.isEnabled()) {
    // if the menu is enabled (i.e. the joy input is owned by the menu),
    // keep the current teleop mode and pause it
    new_mode = mode;
    new_paused = true;
  } else {
    // if the menu is disabled (i.e. the joy input is not owned by the menu),
    // switch the teleop mode based on the menu item selected
    if (model.isSelected("Teleop Mode.Base.Pose")) {
      new_mode = "Base Pose";
    } else if (model.isSelected("Teleop Mode.Base.Twist")) {
      new_mode = "Base Twist";
    } else if (model.isSelected("Teleop Mode.Arm.FK.Pose")) {
      new_mode = "Arm FK Pose";
    } else if (model.isSelected("Teleop Mode.Arm.FK.Twist")) {
      new_mode = "Arm FK Twist";
    } else if (model.isSelected("Teleop Mode.Arm.IK.Pose")) {
      new_mode = "Arm IK Pose";
    } else if (model.isSelected("Teleop Mode.Arm.IK.Twist")) {
      new_mode = "Arm IK Twist";
    } else if (model.isSelected("Teleop Mode.Arm.Hand")) {
      new_mode = "Arm Hand";
    } else {
      new_mode = "None";
    }
    new_paused = false;
  }

  // print the new mode on mode changed
  if (new_mode != mode || new_paused != paused) {
    mode = new_mode;
    paused = new_paused;
    ROS_INFO_STREAM("Teleop mode: " << mode << " (" << (paused ? "paused" : "enabled") << ")");
  }

  // do something
  // doTeleop(mode, paused, *joy);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_integration");
  ros::NodeHandle nh;

  // init the menu tree model
  if (!model.setDescriptionFromParam(nh.resolveName("teleop_menu_description"))) {
    ROS_ERROR_STREAM("Cannot set menu description from the param '"
                     << nh.resolveName("teleop_menu_description"));
    return 1;
  }

  message_filters::Subscriber< sensor_msgs::Joy > joy_sub(nh, "joy", 1);
  message_filters::Subscriber< radial_menu_msgs::State > menu_sub(nh, "teleop_menu_state", 1);
  message_filters::TimeSynchronizer< sensor_msgs::Joy, radial_menu_msgs::State > sync_sub(
      joy_sub, menu_sub, 10);
  sync_sub.registerCallback(onSynchronized);

  ros::spin();

  return 0;
}