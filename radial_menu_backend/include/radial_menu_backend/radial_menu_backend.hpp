#ifndef RADIAL_MENU_BACKEND_RADIAL_MENU_BACKEND_HPP
#define RADIAL_MENU_BACKEND_RADIAL_MENU_BACKEND_HPP

#include <algorithm>
#include <cmath>
#include <cstdint> // for std::int32_t
#include <vector>

#include <radial_menu_msgs/State.h>
#include <sensor_msgs/Joy.h>

namespace radial_menu_backend {

class RadialMenuBackend {
public:
  RadialMenuBackend()
      : allow_multi_selection(false), deselect_on_opening(false), deselect_on_closing(false),
        auto_select(false), open_button(/* PS4's circle*/ 1), select_button(/* PS4's L3 */ 11),
        pointing_axis_v(/* PS4's LEFT Y */ 1), pointing_axis_h(/* PS4's LEFT X */ 0),
        invert_pointing_axis_v(false), invert_pointing_axis_h(false), pointing_axis_threshold(0.5) {
    state.state = State::STATE_CLOSED;
    state.pointed_id = -1;
  }

  virtual ~RadialMenuBackend() {}

  void update(const sensor_msgs::Joy &joy) {
    // create new state instance based on the current state
    State new_state;
    new_state.header.stamp = joy.header.stamp;
    new_state.title = state.title;
    new_state.items = state.items;
    new_state.selected_ids = state.selected_ids;
    new_state.pointed_id = -1; // nothig pointed

    // detemine new state based on open button value
    new_state.state =
        (buttonValue(joy, open_button) > 0) ? State::STATE_OPENED : State::STATE_CLOSED;

    // cancel selections if required
    if (deselect_on_opening && state.state == State::STATE_CLOSED &&
        new_state.state == State::STATE_OPENED) {
      new_state.selected_ids.clear();
    }
    if (deselect_on_closing && state.state == State::STATE_OPENED &&
        new_state.state == State::STATE_CLOSED) {
      new_state.selected_ids.clear();
    }

    // if menu is opened, determine the pointed item based on the pointing axis angle
    if (new_state.state == State::STATE_OPENED) {
      const double value_v(invert_pointing_axis_v ? -axisValue(joy, pointing_axis_v)
                                                  : axisValue(joy, pointing_axis_v));
      const double value_h(invert_pointing_axis_h ? -axisValue(joy, pointing_axis_h)
                                                  : axisValue(joy, pointing_axis_h));
      if (value_v * value_v + value_h * value_h >
          pointing_axis_threshold * pointing_axis_threshold) {
        const double angle(normalizeAngle(std::atan2(value_h, value_v))); // [0, 2*M_PI)
        const double unit_angle(2. * M_PI / new_state.items.size());
        new_state.pointed_id =
            static_cast< int >(std::round(angle / unit_angle)) % new_state.items.size();
      }
    }

    // if an item is pointed & the select button is pressed, update selected items
    if (new_state.pointed_id >= 0 && buttonValue(joy, select_button) > 0) {
      updateSelection(&new_state.selected_ids, new_state.pointed_id, allow_multi_selection);
    }

    // if auto_select is enabled,
    // select the item pointed in the previous state but not in the new state
    if (auto_select && (new_state.state == State::STATE_OPENED && new_state.pointed_id < 0) &&
        state.pointed_id >= 0) {
      updateSelection(&new_state.selected_ids, state.pointed_id, allow_multi_selection);
    }

    // update the current state
    state = new_state;
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

  // update selected items. add a selection if new item was not selected, or deselect.
  static void updateSelection(std::vector< std::int32_t > *const ids, const std::int32_t new_id,
                              const bool allow_multi_selection) {
    if (std::find(ids->begin(), ids->end(), new_id) == ids->end()) {
      if (!allow_multi_selection) {
        ids->clear();
      }
      ids->push_back(new_id);
    } else {
      ids->erase(std::remove(ids->begin(), ids->end(), new_id), ids->end());
    }
  }

public:
  typedef radial_menu_msgs::State State;

  // configs
  bool allow_multi_selection;
  bool deselect_on_opening;
  bool deselect_on_closing;
  bool auto_select;
  int open_button;
  int select_button;
  int pointing_axis_v, pointing_axis_h;
  bool invert_pointing_axis_v, invert_pointing_axis_h;
  double pointing_axis_threshold;

  // menu state
  State state;
};
} // namespace radial_menu_backend

#endif