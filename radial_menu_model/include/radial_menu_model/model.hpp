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

  bool isPointed(const ItemConstPtr &item) const { return !item && item == pointed(); }

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
    const std::vector< ItemConstPtr > items;
    return !item && std::find(items.begin(), items.end(), item) != items.end();
  }

  int sibilingIdByAngle(double angle) const {
    // make the given angle positive, or the returned value may be going to be negative
    while (angle < 0.) {
      angle += 2. * M_PI;
    }
    const int n_sibilings(current_level_->numSibilings());
    const double span_angle(2. * M_PI / n_sibilings);
    return static_cast< int >(std::round(angle / span_angle)) % n_sibilings;
  }

  int pointedSibilingId() const {
    for (int sid = 0; sid < current_level_->numSibilings(); ++sid) {
      if (current_level_->sibiling(sid)->item_id_ == state_.pointed_id) {
        return sid;
      }
    }
    return -1;
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
        item->name_ = *name;
        if (parent_item) {
          item->parent_ = parent_item;
          parent_item->children_.push_back(item);
        }
        items->push_back(item);

        // recursively update the given list
        for (const bpt::ptree::value_type &child_elm : elm.second) {
          if (child_elm.first == "<xmlattr>") {
            continue;
          }
          if (!elementToItems(child_elm, items, item)) {
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
    if (!Internal::elementToItems(root_elm, &new_items)) {
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
    return "<item name=\"Menu\">\n"
           "  <item name=\"Item\" />\n"
           "</item>";
  }

  // *****
  // State
  // *****

  const radial_menu_msgs::State &state() const { return state_; }

  // set new state. also update the current level
  void setState(const radial_menu_msgs::State &new_state) {
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
  }

  void resetState() { setState(defaultState()); }

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

  // can point if the specified sibiling is not pointed
  bool canPoint(const int sid) const {
    const ItemConstPtr sibiling(current_level_->sibiling(sid));
    return sibiling ? !isPointed(sibiling) : false;
  }

  // unpoint all sibilings and point the specified sibiling
  void point(const int sid) {
    if (canPoint(sid)) {
      state_.pointed_id = current_level_->sibiling(sid)->item_id_;
      return;
    }
    throw ros::Exception("Model::point()");
  }

  // can unpoint if the specified sibiling is pointed
  bool canUnpoint(const int sid) const {
    const ItemConstPtr sibiling(current_level_->sibiling(sid));
    return sibiling ? isPointed(sibiling) : false;
  }

  // just unpoint the specified sibiling
  void unpoint(const int sid) {
    if (canUnpoint(sid)) {
      state_.pointed_id = -1;
      return;
    }
    throw ros::Exception("Model::unpoint()");
  }

  // ****************************
  // Leaf selection / deselection
  // ****************************

  // can select if the specified sibiling has no children and is not selected
  bool canSelect(const int sid) const {
    const ItemConstPtr sibiling(current_level_->sibiling(sid));
    return sibiling ? (sibiling->children_.empty() && !isSelected(sibiling)) : false;
  }

  // deselect all sibilings and select the specified sibiling.
  // deselect will be skipped if allow_multi_selection is true.
  void select(const int sid, const bool allow_multi_selection) {
    if (canSelect(sid)) {
      if (!allow_multi_selection) {
        for (const ItemConstPtr &sibiling : current_level_->sibilings()) {
          forceDeselect(sibiling);
        }
      }
      forceSelect(current_level_->sibiling(sid));
      return;
    }
    throw ros::Exception("Model::select()");
  }

  // can deselect if the specified sibiling has no children and is selected
  bool canDeselect(const int sid) const {
    const ItemConstPtr sibiling(current_level_->sibiling(sid));
    return sibiling ? (sibiling->children_.empty() && isSelected(sibiling)) : false;
  }

  // just deselect the specified sibiling
  void deselect(const int sid) {
    if (canDeselect(sid)) {
      forceDeselect(current_level_->sibiling(sid));
      return;
    }
    throw ros::Exception("Model::deselect()");
  }

  // **********************
  // Descending / Ascending
  // **********************

  // can descend if the specified sibiling has a child
  bool canDescend(const int sid) const {
    const ItemConstPtr sibiling(current_level_->sibiling(sid));
    return sibiling ? !sibiling->children_.empty() : false;
  }

  // unpoint and deselect all sibilings, select the specified sibiling
  // and move to the child level of the specified sibiling.
  // deselect will be skipped if allow_multi_selection is true.
  void descend(const int sid, const bool allow_multi_selection) {
    if (canDescend(sid)) {
      state_.pointed_id = -1;
      if (!allow_multi_selection) {
        for (const ItemConstPtr &sibiling : current_level_->sibilings()) {
          forceDeselect(sibiling);
        }
      }
      const ItemConstPtr sibiling(current_level_->sibiling(sid));
      forceSelect(sibiling);
      current_level_ = sibiling->childLevel();
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
          str += item->name_ + "\n";
        } else { // non-root item
          str += std::string(depth * 2, ' ') + "[" + (model->isPointed(item) ? "P" : " ") +
                 (model->isSelected(item) ? "S" : " ") +
                 (item == model->currentLevel() ? "C" : " ") + "] " + item->name_ + "\n";
        }
        for (const ItemConstPtr &child : item->children_) {
          str += toString(model, child);
        }
        return str;
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