#ifndef RADIAL_MENU_MODEL_MODEL_HPP
#define RADIAL_MENU_MODEL_MODEL_HPP

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <radial_menu_model/item.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/param.h>
#include <ros/time.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>

namespace radial_menu_model {

class Model;
typedef boost::shared_ptr< Model > ModelPtr;
typedef boost::shared_ptr< const Model > ModelConstPtr;

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
  bool setDescription(const std::string &new_desc) {
    namespace bpt = boost::property_tree;

    struct Internal {
      static bool elementToItems(const bpt::ptree::value_type &elm,
                                 std::vector< ItemConstPtr > *const items,
                                 int *const sibiling_count,
                                 const ItemPtr &parent_item = ItemPtr()) {
        // is the element name "item"?
        if (elm.first != "item") {
          ROS_ERROR_STREAM("Model::setDescription(): Invalid element '" << elm.first << "'");
          return false;
        }

        // does the element have the attribute "name"?
        const boost::optional< std::string > name(
            elm.second.get_optional< std::string >("<xmlattr>.name"));
        if (!name) {
          ROS_ERROR("Model::setDescription(): No attribute 'name'");
          return false;
        }

        // create an item and append it to the given list
        const ItemPtr item(new Item());
        item->item_id_ = items->size();
        item->sibiling_id_ = *sibiling_count;
        item->name_ = *name;
        if (parent_item) {
          item->parent_ = parent_item;
          parent_item->children_.push_back(item);
        }
        items->push_back(item);
        ++(*sibiling_count);

        // recursively update the given list
        int child_sibiling_count(0);
        for (const bpt::ptree::value_type &child_elm : elm.second) {
          if (child_elm.first == "<xmlattr>") {
            continue;
          }
          if (!elementToItems(child_elm, items, &child_sibiling_count, item)) {
            return false;
          }
        }

        return true;
      }
    };

    // parse the given xml string
    bpt::ptree xml;
    try {
      std::istringstream iss(new_desc);
      bpt::read_xml(iss, xml, bpt::xml_parser::no_comments);
    } catch (const bpt::ptree_error &ex) {
      ROS_ERROR_STREAM("Model::setDescription(): " << ex.what());
      return false;
    }

    // find the root xml element
    if (xml.size() != 1) {
      ROS_ERROR("Model::setDescription(): Non unique root element in xml");
      return false;
    }
    const bpt::ptree::value_type &root_elm(xml.front());

    // populate items in the item tree
    std::vector< ItemConstPtr > new_items;
    int root_sibiling_count(0);
    if (!Internal::elementToItems(root_elm, &new_items, &root_sibiling_count)) {
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
    std::string new_desc;
    if (ros::param::get(param_name, new_desc)) {
      return setDescription(new_desc);
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
    current_level_ = current_level_->root()->childLevel();
    for (const std::int32_t iid : state_.selected_ids) {
      if (iid >= 0 && iid < items_.size()) {
        const ItemConstPtr item(items_[iid]);
        const ItemConstPtr level(item->children_.empty() ? item->sibilingLevel()
                                                         : item->childLevel());
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
      state_.pointed_id = item->item_id_;
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

  // can select if the given item is in the current level, has no children, and is not selected
  bool canSelect(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && item->children_.empty() &&
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

  // can deselect if the given item is in the current level, has no children, and is selected
  bool canDeselect(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && item->children_.empty() &&
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

  // can descend if the given item is in the current level and has a child
  bool canDescend(const ItemConstPtr &item) const {
    return item && item->sibilingLevel() == current_level_ && !item->children_.empty();
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
  bool canAscend() const { return current_level_->parent() != current_level_->root(); }

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
      static std::string toString(const Model *const model, const ItemConstPtr &item) {
        const int depth(item->depth());
        std::string str;
        if (depth <= 0) { // root item
          str += item->name_ + " " + itemIdStr(item) + "\n";
        } else { // non-root item
          str += std::string(depth * 2, ' ') + itemStateStr(model, item) + " " + item->name_ + " " +
                 itemIdStr(item) + "\n";
        }
        for (const ItemConstPtr &child : item->children_) {
          str += toString(model, child);
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
        str << "(i" << item->item_id_ << "-s" << item->sibiling_id_ << ")";
        return str.str();
      }
    };

    return Internal::toString(this, current_level_->root());
  }

protected:
  // ************
  // Internal use
  // ************

  void forceSelect(const ItemConstPtr &item) {
    std::vector< std::int32_t > &ids(state_.selected_ids);
    if (std::find(ids.begin(), ids.end(), item->item_id_) == ids.end()) {
      ids.push_back(item->item_id_);
    }
  }

  void forceDeselect(const ItemConstPtr &item) {
    std::vector< std::int32_t > &ids(state_.selected_ids);
    ids.erase(std::remove(ids.begin(), ids.end(), item->item_id_), ids.end());
  }

protected:
  std::vector< ItemConstPtr > items_;
  ItemConstPtr current_level_;
  radial_menu_msgs::State state_;
};
} // namespace radial_menu_model

#endif