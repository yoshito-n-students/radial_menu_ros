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
} // namespace radial_menu_msgs

#endif