#ifndef RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP
#define RADIAL_MENU_BACKEND_BACKEND_CONTROLLER_HPP

#include <cmath>
#include <limits>
#include <memory>

#include <radial_menu_backend/backend_config.hpp>
#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <sensor_msgs/Joy.h>

namespace radial_menu_backend {

class BackendController;
typedef std::shared_ptr< BackendController > BackendControllerPtr;
typedef std::shared_ptr< const BackendController > BackendControllerConstPtr;

class BackendController {
public:
  BackendController(const radial_menu_model::ModelPtr &model, const BackendConfig &config)
      : model_(model), enable_was_pressed_(false), select_was_pressed_(false),
        ascend_was_pressed_(false), config_(config) {}

  virtual ~BackendController() {}

  radial_menu_msgs::StatePtr update(const sensor_msgs::Joy &joy) {
    // reset the menu based on enable/disable state if required
    const bool enable_is_pressed(buttonValue(joy, config_.enable_button) > 0);
    if ((config_.reset_on_enabling && !enable_was_pressed_ && enable_is_pressed) ||
        (config_.reset_on_disabling && enable_was_pressed_ && !enable_is_pressed)) {
      model_->resetState();
    }

    // enable/disable the menu
    // (resetState() disables the menu so setEnabled() must be called after resetState())
    model_->setEnabled(enable_is_pressed);

    // unpoint if possible
    const radial_menu_model::ItemConstPtr last_pointed_item(model_->pointed());
    if (model_->canUnpoint(last_pointed_item)) {
      model_->unpoint(last_pointed_item);
    }

    // do remaining operations if the menu is enabled
    const bool select_is_pressed(buttonValue(joy, config_.select_button) > 0);
    const bool ascend_is_pressed(buttonValue(joy, config_.ascend_button) > 0);
    if (enable_is_pressed) {
      // update the pointing item
      const double pointing_angle(pointingAngle(joy));
      if (!std::isnan(pointing_angle)) {
        const radial_menu_model::ItemConstPtr item_to_point(
            model_->sibilingByAngle(pointing_angle));
        if (model_->canPoint(item_to_point)) {
          model_->point(item_to_point);
        }
      }

      // if an item is pointed and the select button is newly pressed, select the pointed item,
      // else if auto-select is enabled and no item is pointed, select the last pointed item
      const radial_menu_model::ItemConstPtr pointed_item(model_->pointed());
      if (pointed_item && select_is_pressed && !select_was_pressed_) {
        adaptiveSelect(pointed_item);
      } else if (config_.auto_select && std::isnan(pointing_angle) && last_pointed_item) {
        adaptiveSelect(last_pointed_item);
      }

      // if the ascend button is newly pressed, ascend from the current level
      if (ascend_is_pressed && !ascend_was_pressed_) {
        if (model_->canAscend()) {
          model_->ascend();
        }
      }
    }

    // update memos
    enable_was_pressed_ = enable_is_pressed;
    select_was_pressed_ = select_is_pressed;
    ascend_was_pressed_ = ascend_is_pressed;

    return model_->exportState(joy.header.stamp);
  }

protected:
  // utility functions

  double pointingAngle(const sensor_msgs::Joy &joy) const {
    const double value_v(config_.invert_pointing_axis_v ? -axisValue(joy, config_.pointing_axis_v)
                                                        : axisValue(joy, config_.pointing_axis_v));
    const double value_h(config_.invert_pointing_axis_h ? -axisValue(joy, config_.pointing_axis_h)
                                                        : axisValue(joy, config_.pointing_axis_h));
    return (value_v * value_v + value_h * value_h >=
            config_.pointing_axis_threshold * config_.pointing_axis_threshold)
               ? std::atan2(value_h, value_v)
               : std::numeric_limits< double >::quiet_NaN();
  }

  void adaptiveSelect(const radial_menu_model::ItemConstPtr &item) {
    if (model_->canSelect(item)) {
      model_->select(item, config_.allow_multi_selection);
    } else if (model_->canDeselect(item)) {
      model_->deselect(item);
    } else if (model_->canDescend(item)) {
      model_->descend(item, config_.allow_multi_selection);
    }
  }

  // return button value without id range error
  static int buttonValue(const sensor_msgs::Joy &joy, const int bid) {
    return (bid >= 0 && bid < joy.buttons.size()) ? joy.buttons[bid] : 0;
  }

  // return axis value without id range error
  static double axisValue(const sensor_msgs::Joy &joy, const int aid) {
    return (aid >= 0 && aid < joy.axes.size()) ? joy.axes[aid] : 0.;
  }

protected:
  radial_menu_model::ModelPtr model_;

  // memo
  bool enable_was_pressed_, select_was_pressed_, ascend_was_pressed_;

  const BackendConfig config_;
};
} // namespace radial_menu_backend

#endif