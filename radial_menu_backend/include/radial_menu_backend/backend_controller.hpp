#ifndef RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP
#define RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint> // for std::int32_t
#include <vector>

#include <radial_menu_backend/menu.hpp>
#include <radial_menu_backend/backend_config.hpp>
#include <radial_menu_msgs/State.h>
#include <sensor_msgs/Joy.h>

namespace radial_menu_backend {

class BackendController;
typedef boost::shared_ptr< BackendController > BackendControllerPtr;
typedef boost::shared_ptr< const BackendController > BackendControllerConstPtr;

class BackendController {
public:
  BackendController(const MenuPtr menu, const BackendConfig &config)
      : menu_(menu), was_enabled_(false), last_pointed_(), select_was_pressed_(false),
        ascend_was_pressed_(false), config_(config) {
    menu_ = menu_->reset(); // reset and move to the root
    if (menu_->canDescend()) {
      menu_ = menu_->descend(); // descend to the first level
    }
  }

  virtual ~BackendController() {}

  radial_menu_msgs::StatePtr update(const sensor_msgs::Joy &joy) {
    // reset the menu based on enable/disable state if required
    const bool is_enabled(buttonValue(joy, config_.enable_button) > 0);
    if ((config_.reset_on_enabling && !was_enabled_ && is_enabled) ||
        (config_.reset_on_disabling && was_enabled_ && !is_enabled)) {
      menu_ = menu_->reset(); // reset and move to the root
      if (menu_->canDescend()) {
        menu_ = menu_->descend(); // descend to the first level
      }
    }

    // unpoint all items before updating pointed item
    menu_->unpointAll();

    // if menu is enabled, determine the pointed item based on the pointing axis angle
    MenuPtr pointed;
    if (is_enabled) {
      const double value_v(config_.invert_pointing_axis_v
                               ? -axisValue(joy, config_.pointing_axis_v)
                               : axisValue(joy, config_.pointing_axis_v));
      const double value_h(config_.invert_pointing_axis_h
                               ? -axisValue(joy, config_.pointing_axis_h)
                               : axisValue(joy, config_.pointing_axis_h));
      if (value_v * value_v + value_h * value_h >
          config_.pointing_axis_threshold * config_.pointing_axis_threshold) {
        const double point_angle(normalizeAngle(std::atan2(value_h, value_v))); // [0, 2*M_PI)
        const std::size_t n_sibilings(menu_->numSibilings());
        const double span_angle(2. * M_PI / n_sibilings);
        const std::size_t pointed_id(
            static_cast< std::size_t >(std::round(point_angle / span_angle)) % n_sibilings);
        pointed = menu_->sibiling(pointed_id);
        pointed->point();
      }
    }

    // if the select button is pressed and an item is pointed, select the pointed item
    const bool select_is_pressed(buttonValue(joy, config_.select_button) > 0);
    if (is_enabled && pointed && select_is_pressed && !select_was_pressed_) {
      if (pointed->canSelect()) {
        pointed->select(config_.allow_multi_selection);
      } else if (pointed->canDeselect()) {
        pointed->deselect();
      } else if (pointed->canDescend()) {
        menu_ = pointed->descend();
      }
    }

    // if auto-select is enabled and no item is pointed, select the last pointed item
    if (config_.auto_select && (is_enabled && !pointed) && (was_enabled_ && last_pointed_)) {
      if (last_pointed_->canSelect()) {
        last_pointed_->select(config_.allow_multi_selection);
      } else if (last_pointed_->canDeselect()) {
        last_pointed_->deselect();
      } else if (last_pointed_->canDescend()) {
        menu_ = last_pointed_->descend();
      }
    }

    // if the ascend button is pressed, ascend from the current level
    const bool ascend_is_pressed(buttonValue(joy, config_.ascend_button) > 0);
    if (is_enabled && ascend_is_pressed && !ascend_was_pressed_) {
      if (menu_->canAscend()) {
        menu_ = menu_->ascend();
      }
      // if ascended to the root, descend to the first level
      if (menu_->isRoot() && menu_->canDescend()) {
        menu_ = menu_->descend();
      }
    }

    // update memos
    was_enabled_ = is_enabled;
    last_pointed_ = pointed;
    select_was_pressed_ = select_is_pressed;
    ascend_was_pressed_ = ascend_is_pressed;

    return menu_->toState(joy.header.stamp, is_enabled);
  }

protected:
  // utility static functions

  // return button value without id range error
  static int buttonValue(const sensor_msgs::Joy &joy, const int id) {
    return (id >= 0 && id < joy.buttons.size()) ? joy.buttons[id] : 0;
  }

  // return axis value without id range error
  static double axisValue(const sensor_msgs::Joy &joy, const int id) {
    return (id >= 0 && id < joy.axes.size()) ? joy.axes[id] : 0.;
  }

  // normalize angle to [0, 2 * M_PI)
  static double normalizeAngle(double angle) {
    while (angle >= 2. * M_PI) {
      angle -= 2. * M_PI;
    }
    while (angle < 0.) {
      angle += 2. * M_PI;
    }
    return angle;
  }

protected:
  MenuPtr menu_;

  // memo
  bool was_enabled_;
  MenuPtr last_pointed_;
  bool select_was_pressed_, ascend_was_pressed_;

  const BackendConfig config_;
};
} // namespace radial_menu_backend

#endif