#ifndef RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP
#define RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP

#include <cmath>

#include <radial_menu_backend/backend_config.hpp>
#include <radial_menu_backend/menu.hpp>
#include <radial_menu_msgs/State.h>
#include <sensor_msgs/Joy.h>

namespace radial_menu_backend {

class BackendController;
typedef boost::shared_ptr< BackendController > BackendControllerPtr;
typedef boost::shared_ptr< const BackendController > BackendControllerConstPtr;

class BackendController {
public:
  BackendController(const MenuPtr menu, const BackendConfig &config)
      : menu_(menu), enable_was_pressed_(false), select_was_pressed_(false),
        ascend_was_pressed_(false), config_(config) {
    menu_->reset();
  }

  virtual ~BackendController() {}

  radial_menu_msgs::StatePtr update(const sensor_msgs::Joy &joy) {
    // reset the menu based on enable/disable state if required
    const bool enable_is_pressed(buttonValue(joy, config_.enable_button) > 0);
    if ((config_.reset_on_enabling && !enable_was_pressed_ && enable_is_pressed) ||
        (config_.reset_on_disabling && enable_was_pressed_ && !enable_is_pressed)) {
      menu_->reset();
    }

    // unpoint if possible
    const int last_pointed_id(menu_->pointedId());
    if (menu_->canUnpoint(last_pointed_id)) {
      menu_->unpoint(last_pointed_id);
    }

    // do remaining operations if the menu is enabled
    const bool select_is_pressed(buttonValue(joy, config_.select_button) > 0);
    const bool ascend_is_pressed(buttonValue(joy, config_.ascend_button) > 0);
    if (enable_is_pressed) {
      // if axes is enough tilted, point an item
      const double value_v(config_.invert_pointing_axis_v
                               ? -axisValue(joy, config_.pointing_axis_v)
                               : axisValue(joy, config_.pointing_axis_v));
      const double value_h(config_.invert_pointing_axis_h
                               ? -axisValue(joy, config_.pointing_axis_h)
                               : axisValue(joy, config_.pointing_axis_h));
      if (value_v * value_v + value_h * value_h >=
          config_.pointing_axis_threshold * config_.pointing_axis_threshold) {
        const int id(menu_->idByAngle(std::atan2(value_h, value_v)));
        if (menu_->canPoint(id)) {
          menu_->point(id);
        }
      }

      // if an item is pointed and the select button is newly pressed, select the pointed item,
      // else if auto-select is enabled and no item is pointed, select the last pointed item
      const int pointed_id(menu_->pointedId());
      if (pointed_id >= 0 && select_is_pressed && !select_was_pressed_) {
        adaptiveSelect(pointed_id);
      } else if (config_.auto_select && pointed_id < 0 && last_pointed_id >= 0) {
        adaptiveSelect(last_pointed_id);
      }

      // if the ascend button is newly pressed, ascend from the current level
      if (ascend_is_pressed && !ascend_was_pressed_) {
        if (menu_->canAscend()) {
          menu_->ascend();
        }
      }
    }

    // update memos
    enable_was_pressed_ = enable_is_pressed;
    select_was_pressed_ = select_is_pressed;
    ascend_was_pressed_ = ascend_is_pressed;

    return menu_->toState(joy.header.stamp, enable_is_pressed);
  }

protected:
  // utility functions

  void adaptiveSelect(const int id) {
    if (menu_->canSelect(id)) {
      menu_->select(id, config_.allow_multi_selection);
    } else if (menu_->canDeselect(id)) {
      menu_->deselect(id);
    } else if (menu_->canDescend(id)) {
      menu_->descend(id, config_.allow_multi_selection);
    }
  }

  // return button value without id range error
  static int buttonValue(const sensor_msgs::Joy &joy, const int id) {
    return (id >= 0 && id < joy.buttons.size()) ? joy.buttons[id] : 0;
  }

  // return axis value without id range error
  static double axisValue(const sensor_msgs::Joy &joy, const int id) {
    return (id >= 0 && id < joy.axes.size()) ? joy.axes[id] : 0.;
  }

protected:
  MenuPtr menu_;

  // memo
  bool enable_was_pressed_, select_was_pressed_, ascend_was_pressed_;

  const BackendConfig config_;
};
} // namespace radial_menu_backend

#endif