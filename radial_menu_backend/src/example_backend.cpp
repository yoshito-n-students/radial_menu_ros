#include <radial_menu_backend/backend.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

radial_menu_backend::Backend backend;
ros::Publisher state_pub;

// callback
void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
  // update menu state and publish
  backend.update(*joy);
  state_pub.publish(backend.state);

  /*
  // defer joy input to other objects as you want
  if (backend.isClosed()) {
    if (backend.isSelected("foo")) {
      // foo.process(joy);
    } else if (backend.isSelected("bar")) {
      // bar.process(joy);
    }
  }
  */
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_backend");
  ros::NodeHandle nh, pnh("~");

  // load menu items
  pnh.getParam("title", backend.state.title);
  pnh.getParam("items", backend.state.items);

  // load other configs from params
  pnh.getParam("allow_multi_selection", backend.allow_multi_selection);
  pnh.getParam("deselect_on_opening", backend.deselect_on_opening);
  pnh.getParam("deselect_on_closing", backend.deselect_on_closing);
  pnh.getParam("auto_select", backend.auto_select);
  pnh.getParam("open_button", backend.open_button);
  pnh.getParam("select_button", backend.select_button);
  pnh.getParam("pointing_axis_v", backend.pointing_axis_v);
  pnh.getParam("invert_pointing_axis_v", backend.invert_pointing_axis_v);
  pnh.getParam("pointing_axis_h", backend.pointing_axis_h);
  pnh.getParam("invert_pointing_axis_h", backend.invert_pointing_axis_h);
  pnh.getParam("pointing_axis_threshold", backend.pointing_axis_threshold);

  //
  state_pub = nh.advertise< radial_menu_msgs::State >("radial_menu_state", 1, true);
  ros::Subscriber joy_sub(nh.subscribe("joy", 1, onJoyRecieved));

  //
  ros::spin();

  return 0;
}