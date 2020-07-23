#ifndef RADIAL_MENU_BACKEND_MENU_CONFIG_HPP
#define RADIAL_MENU_BACKEND_MENU_CONFIG_HPP

#include <string>

#include <ros/node_handle.h>

namespace radial_menu_backend {

struct MenuConfig {
  MenuConfig()
      : allow_multi_selection(false), reset_on_enabling(false), reset_on_disabling(false),
        auto_select(false), enable_button(/* PS4's circle*/ 1), select_button(/* PS4's R1 */ 5),
        ascend_button(/* PS4's L1 */ 4), pointing_axis_v(/* PS4's LEFT Y */ 1),
        pointing_axis_h(/* PS4's LEFT X */ 0), invert_pointing_axis_v(false),
        invert_pointing_axis_h(false), pointing_axis_threshold(0.5) {}

  static MenuConfig fromParamNs(const std::string &ns) {
    ros::NodeHandle nh(ns);
    MenuConfig config;
    nh.getParam("allow_multi_selection", config.allow_multi_selection);
    nh.getParam("reset_on_enabling", config.reset_on_enabling);
    nh.getParam("reset_on_disabling", config.reset_on_disabling);
    nh.getParam("auto_select", config.auto_select);
    nh.getParam("enable_button", config.enable_button);
    nh.getParam("select_button", config.select_button);
    nh.getParam("ascend_button", config.ascend_button);
    nh.getParam("pointing_axis_v", config.pointing_axis_v);
    nh.getParam("invert_pointing_axis_v", config.invert_pointing_axis_v);
    nh.getParam("pointing_axis_h", config.pointing_axis_h);
    nh.getParam("invert_pointing_axis_h", config.invert_pointing_axis_h);
    nh.getParam("pointing_axis_threshold", config.pointing_axis_threshold);
    return config;
  }

  bool allow_multi_selection;
  bool reset_on_enabling;
  bool reset_on_disabling;
  bool auto_select;
  int enable_button;
  int select_button;
  int ascend_button;
  int pointing_axis_v, pointing_axis_h;
  bool invert_pointing_axis_v, invert_pointing_axis_h;
  double pointing_axis_threshold;
};
} // namespace radial_menu_backend

#endif