#ifndef RADIAL_MENU_MODEL_MODEL_HPP
#define RADIAL_MENU_MODEL_MODEL_HPP

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <radial_menu_model/item.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/param.h>
#include <ros/time.h>

namespace radial_menu_model {

class Model;
typedef std::shared_ptr< Model > ModelPtr;
typedef std::shared_ptr< const Model > ModelConstPtr;

class Model {
public:
  Model() {
    resetDescription();
    resetState();
  }

  virtual ~Model() {}

  // **************
  // Current status
  // **************

  ItemConstPtr root() const { return current_level_->root(); }

  ItemConstPtr currentLevel() const { return current_level_; }

  ItemConstPtr pointed() const {
    return (state_.pointed_id >= 0 && state_.pointed_id < items_.size()) ? items_[state_.pointed_id]
                                                                         : ItemConstPtr();
  }

  bool isPointed(const ItemConstPtr &item) const { return item && item == pointed(); }

  bool isPointed(const std::string &path, const char separator = '.') const {
    const ItemConstPtr p(pointed());
    return p ? (path == p->path(separator)) : false;
  }

  std::vector< ItemConstPtr > selected() const {
    std::vector< ItemConstPtr > items;
    for (const std::int32_t iid : state_.selected_ids) {
      if (iid >= 0 && iid < items_.size()) {
        items.push_back(items_[iid]);
      }
    }
    return items;
  }

  bool isSelected(const ItemConstPtr &item) const {
    if (!item) {
      return false;
    }
    for (const std::int32_t iid : state_.selected_ids) {
      if (iid >= 0 && iid < items_.size() && item == items_[iid]) {
        return true;
      }
    }
    return false;
  }

  bool isSelected(const std::string &path, const char separator = '.') const {
    for (const std::int32_t iid : state_.selected_ids) {
      if (iid >= 0 && iid < items_.size() && path == items_[iid]->path(separator)) {
        return true;
      }
    }
    return false;
  }

  ItemConstPtr sibilingByAngle(double angle) const {
    // make the given angle positive, or the returned value may be going to be negative
    while (angle < 0.) {
      angle += 2. * M_PI;
    }
    const int n_sibilings(current_level_->numSibilings());
    const double span_angle(2. * M_PI / n_sibilings);
    const int sid(static_cast< int >(std::round(angle / span_angle)) % n_sibilings);
    return current_level_->sibiling(sid);
  }

  // ***********
  // Description
  // ***********

  // set new model tree description. also rests the state
  bool setDescription(const std::string &desc) {
    // populate items in the item tree
    const std::vector< ItemConstPtr > new_items(Item::itemsFromDescription(desc));
    if (new_items.empty()) {
      ROS_ERROR("Model::setDescription(): No items");
      return false;
    }

    // set the initial level of the model
    const ItemConstPtr new_current_level(new_items.front()->childLevel());
    if (!new_current_level) {
      ROS_ERROR("Model::setDescription(): No children of the root item");
      return false;
    }

    // set new description (also reset the state)
    items_ = new_items;
    current_level_ = new_current_level;
    state_ = defaultState();
    return true;
  }

  // set new description obtained from a ROS param server
  bool setDescriptionFromParam(const std::string &param_name) {
    std::string desc;
    if (ros::param::get(param_name, desc)) {
      return setDescription(desc);
    } else {
      ROS_ERROR_STREAM("Model::setDescriptionFromParam(): Cannot get the param '" << param_name
                                                                                  << "'");
      return false;
    }
  }

  bool resetDescription() { return setDescription(defaultDescription()); }

  static std::string defaultDescription() {
    // this is useless as an actual menu, but keeps items_ & current_level_ valid
    return "<item name=\"Menu\">\n"
           "  <item name=\"Item\" />\n"
           "</item>";
  }

  // *****
  // State
  // *****

  radial_menu_msgs::StatePtr exportState(const ros::Time stamp = ros::Time::now()) const {
    radial_menu_msgs::StatePtr state(new radial_menu_msgs::State(state_));
    state->header.stamp = stamp;
    return state;
  }

  // set new state. also update the current level
  bool setState(const radial_menu_msgs::State &new_state) {
    state_ = new_state;

    // update the current level by moving to the deepest level of selected items or its children
    current_level_ = items_.front()->childLevel();
    for (const std::int32_t iid : state_.selected_ids) {
      if (iid >= 0 && iid < items_.size()) {
        const ItemConstPtr item(items_[iid]);
        ItemConstPtr level(item->childLevel());
        if (!level) {
          level = item->sibilingLevel();
        }
        if (level->depth() > current_level_->depth()) {
          current_level_ = level;
        }
      }
    }

    return true;
  }

  bool resetState() { return setState(defaultState()); }

  static radial_menu_msgs::State defaultState() {
    radial_menu_msgs::State state;
    state.is_enabled = false;
    state.pointed_id = -1;
    return state;
  }

  // ********
  // Enabling
  // ********

  bool isEnabled() const { return state_.is_enabled; }

  void setEnabled(const bool is_enabled) { state_.is_enabled = is_enabled; }

  // *********************
  // Pointing / Unpointing
  // *********************

  // can point if the given item is in the current level and not pointed
  bool canPoint(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && !isPointed(item);
  }

  // move the point to the given item
  void point(const ItemConstPtr &item) {
    if (canPoint(item)) {
      state_.pointed_id = item->itemId();
      return;
    }
    throw ros::Exception("Model::point()");
  }

  // can unpoint if the given item is in the current level and pointed
  bool canUnpoint(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && isPointed(item);
  }

  // just unpoint the given item
  void unpoint(const ItemConstPtr &item) {
    if (canUnpoint(item)) {
      state_.pointed_id = -1;
      return;
    }
    throw ros::Exception("Model::unpoint()");
  }

  // ****************************
  // Leaf selection / deselection
  // ****************************

  // can select if the given item is in the current level, has no child level, and is not selected
  bool canSelect(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && !item->childLevel() &&
           !isSelected(item);
  }

  // deselect all sibilings and select the given item.
  // deselect will be skipped if allow_multi_selection is true.
  void select(const ItemConstPtr &item, const bool allow_multi_selection) {
    if (canSelect(item)) {
      if (!allow_multi_selection) {
        for (const ItemConstPtr &sibiling : current_level_->sibilings()) {
          forceDeselect(sibiling);
        }
      }
      forceSelect(item);
      return;
    }
    throw ros::Exception("Model::select()");
  }

  // can deselect if the given item is in the current level, has no child level, and is selected
  bool canDeselect(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && !item->childLevel() &&
           isSelected(item);
  }

  // just deselect the given item
  void deselect(const ItemConstPtr &item) {
    if (canDeselect(item)) {
      forceDeselect(item);
      return;
    }
    throw ros::Exception("Model::deselect()");
  }

  // **********************
  // Descending / Ascending
  // **********************

  // can descend if the given item is in the current level and has a child level
  bool canDescend(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && item->childLevel();
  }

  // unpoint and deselect all sibilings, select the given item
  // and move to the child level of the given item.
  // deselect will be skipped if allow_multi_selection is true.
  void descend(const ItemConstPtr &item, const bool allow_multi_selection) {
    if (canDescend(item)) {
      state_.pointed_id = -1;
      if (!allow_multi_selection) {
        for (const ItemConstPtr &sibiling : current_level_->sibilings()) {
          forceDeselect(sibiling);
        }
      }
      forceSelect(item);
      current_level_ = item->childLevel();
      return;
    }
    throw ros::Exception("Model::descend()");
  }

  // can ascend if the current level is not the first
  bool canAscend() const { return current_level_->depth() >= 2; }

  // unpoint and deselect all sibilings, deselect and move to the parent level
  void ascend() {
    if (canAscend()) {
      state_.pointed_id = -1;
      for (const ItemConstPtr &sibiling : current_level_->sibilings()) {
        forceDeselect(sibiling);
      }
      forceDeselect(current_level_->parent());
      current_level_ = current_level_->parentLevel();
      return;
    }
    throw ros::Exception("Model::ascend()");
  }

  // *****
  // Debug
  // *****

  std::string toString() const {
    struct Internal {
      static std::string toString(const Model *const model, const ItemConstPtr &item,
                                  const int n_indent = 0) {
        std::string str(n_indent, ' ');
        if (item) {
          if (item->depth() > 0) {
            str += itemStateStr(model, item) + " ";
          }
          str += item->name() + " " + itemIdStr(item) + "\n";
          for (const ItemConstPtr &child : item->children()) {
            str += toString(model, child, n_indent + 2);
          }
        } else {
          str += "      -\n";
        }
        return str;
      }

      static std::string itemStateStr(const Model *const model, const ItemConstPtr &item) {
        std::ostringstream str;
        str << "[" << (model->isPointed(item) ? "P" : " ") << (model->isSelected(item) ? "S" : " ")
            << (item == model->currentLevel() ? "C" : " ") << "]";
        return str.str();
      }

      static std::string itemIdStr(const ItemConstPtr &item) {
        std::ostringstream str;
        str << "(i" << item->itemId() << "-d" << item->depth() << ")";
        return str.str();
      }
    };

    return Internal::toString(this, items_.front());
  }

protected:
  // ************
  // Internal use
  // ************

  void forceSelect(const ItemConstPtr &item) {
    if (item) {
      std::vector< std::int32_t > &ids(state_.selected_ids);
      if (std::find(ids.begin(), ids.end(), item->itemId()) == ids.end()) {
        ids.push_back(item->itemId());
      }
    }
  }

  void forceDeselect(const ItemConstPtr &item) {
    if (item) {
      std::vector< std::int32_t > &ids(state_.selected_ids);
      ids.erase(std::remove(ids.begin(), ids.end(), item->itemId()), ids.end());
    }
  }

protected:
  std::vector< ItemConstPtr > items_;
  ItemConstPtr current_level_;
  radial_menu_msgs::State state_;
};
} // namespace radial_menu_model

#endif