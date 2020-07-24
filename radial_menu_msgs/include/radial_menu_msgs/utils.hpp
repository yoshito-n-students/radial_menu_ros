#ifndef RADIAL_MENU_MSGS_UTILS_HPP
#define RADIAL_MENU_MSGS_UTILS_HPP

#include <cstdint>
#include <string>

#include <radial_menu_msgs/State.h>

namespace radial_menu_msgs {

static inline State disabledState() {
  State state;
  state.is_enabled = false;
  state.pointed_id = -1;
  return state;
}

static inline StatePtr disabledStatePtr() {
  StatePtr state(new State());
  state->is_enabled = false;
  state->pointed_id = -1;
  return state;
}

static inline bool isSelected(const State &state, const std::string &item) {
  for (const std::int32_t id : state.selected_ids) {
    if (id >= 0 && id < state.items.size() && state.items[id] == item) {
      return true;
    }
  }
  return false;
}

static inline bool isPointed(const State &state, const std::string &item) {
  return state.pointed_id >= 0 && state.pointed_id < state.items.size() &&
         state.items[state.pointed_id] == item;
}

static inline bool changed(const State &a, const State &b) {
  return a.title != b.title || a.is_enabled != b.is_enabled || a.items != b.items ||
         a.widths != b.widths || a.pointed_id != b.pointed_id || a.selected_ids != b.selected_ids;
}

static inline bool changed(const StateConstPtr &a, const StateConstPtr &b) {
  return (a && !b) || (!a && b) || (a && b && changed(*a, *b));
}
} // namespace radial_menu_msgs

#endif