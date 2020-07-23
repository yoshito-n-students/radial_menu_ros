#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <radial_menu_msgs/utils.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>

void processJoy(const sensor_msgs::Joy &joy) {
  // add your code here
}

void onSynchronized(const sensor_msgs::JoyConstPtr &joy, const radial_menu_msgs::StateConstPtr &menu) {
  // if the menu does not own the joy input and the item of interest is selected,
  // process the joy message
  if (!menu->is_enabled && radial_menu_msgs::isSelected(*menu, "foo")) {
    processJoy(*joy);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_integration");
  ros::NodeHandle nh;

  message_filters::Subscriber< sensor_msgs::Joy > joy_sub(nh, "joy", 1);
  message_filters::Subscriber< radial_menu_msgs::State > menu_sub(nh, "radial_menu_state", 1);
  message_filters::TimeSynchronizer< sensor_msgs::Joy, radial_menu_msgs::State > sync_sub(
      joy_sub, menu_sub, 10);
  sync_sub.registerCallback(onSynchronized);

  ros::spin();

  return 0;
}