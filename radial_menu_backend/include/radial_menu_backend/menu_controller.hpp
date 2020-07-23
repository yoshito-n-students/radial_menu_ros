#ifndef RADIAL_MENU_BACKEND_MENU_CONTROLLER_HPP
#define RADIAL_MENU_BACKEND_MENU_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint> // for std::int32_t
#include <vector>

#include <radial_menu_backend/menu.hpp>
#include <radial_menu_backend/menu_config.hpp>
#include <radial_menu_msgs/State.h>
#include <sensor_msgs/Joy.h>

namespace radial_menu_backend {

class MenuController;
typedef boost::shared_ptr< MenuController > MenuControllerPtr;
typedef boost::shared_ptr< const MenuController > MenuControllerConstPtr;

class MenuController {
public:
  MenuController(const MenuPtr menu, const MenuConfig &config)
      : menu_(menu), was_opened_(false), config_(config) {
    menu_ = menu_->reset(); // reset and move to the root
    if (menu_->canDescend()) {
      menu_ = menu_->descend(); // descend to the first level
    }
  }

  virtual ~MenuController() {}

  radial_menu_msgs::StatePtr update(const sensor_msgs::Joy &joy) {
    // determine open/close state
    const bool is_opened(buttonValue(joy, config_.open_button) > 0);

    // reset the menu if required
    if ((config_.reset_on_opening && !was_opened_ && is_opened) ||
        (config_.reset_on_closing && was_opened_ && !is_opened)) {
      menu_ = menu_->reset(); // reset and move to the root
      if (menu_->canDescend()) {
        menu_ = menu_->descend(); // descend to the first level
      }
    }

    // unpoint all items before updating pointed item
    MenuPtr pointed;
    menu_->unpointAll();

    // if menu is opened, determine the pointed item based on the pointing axis angle
    if (is_opened) {
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
    if (is_opened && pointed && buttonValue(joy, config_.select_button) > 0) {
      if (pointed->canSelect()) {
        pointed->select(config_.allow_multi_selection);
      } else if (pointed->canDeselect()) {
        pointed->deselect();
      } else if (pointed->canDescend()) {
        menu_ = pointed->descend();
      }
    }

    // if auto-select is enabled and no item is pointed, select the last pointed item
    if (is_opened && !pointed && last_pointed_ && config_.auto_select) {
      if (last_pointed_->canSelect()) {
        last_pointed_->select(config_.allow_multi_selection);
      } else if (last_pointed_->canDeselect()) {
        last_pointed_->deselect();
      } else if (last_pointed_->canDescend()) {
        menu_ = last_pointed_->descend();
      }
    }

    // if the ascend button is pressed, ascend from the current level
    if (is_opened && buttonValue(joy, config_.ascend_button) > 0) {
      if (menu_->canAscend()) {
        menu_ = menu_->ascend();
      }
      // if ascended to the root, descend to the first level
      if (menu_->isRoot() && menu_->canDescend()) {
        menu_ = menu_->descend();
      }
    }

    // update memos
    was_opened_ = is_opened;
    last_pointed_ = pointed;

    return menu_->toState(joy.header.stamp, is_opened);
  }

private:
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
  bool was_opened_;
  MenuPtr last_pointed_;
  const MenuConfig config_;
};
} // namespace radial_menu_backend

#endif