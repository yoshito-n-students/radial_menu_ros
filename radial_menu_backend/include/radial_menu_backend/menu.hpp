#ifndef RADIAL_MENU_BACKEND_MENU_HPP
#define RADIAL_MENU_BACKEND_MENU_HPP

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/param.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace radial_menu_backend {

class Menu;
typedef boost::shared_ptr< Menu > MenuPtr;
typedef boost::shared_ptr< const Menu > MenuConstPtr;

class Menu {
protected:
  Menu() {}

public:
  // id utilities

  int idByAngle(double angle) const {
    // make the given angle positive, or the returned value may be going to be negative
    while (angle < 0.) {
      angle += 2. * M_PI;
    }
    const int n_sibilings(sibilingsOf(current_level_).size());
    const double span_angle(2. * M_PI / n_sibilings);
    return static_cast< int >(std::round(angle / span_angle)) % n_sibilings;
  }

  int pointedId() const {
    const std::vector< ItemPtr > sibilings(sibilingsOf(current_level_));
    for (int id = 0; id < sibilings.size(); ++id) {
      if (sibilings[id]->is_pointed) {
        return id;
      }
    }
    return -1;
  }

  // pointing

  // can point if the specified sibiling is not pointed
  bool canPoint(const int id) const {
    const ItemConstPtr sibiling(sibilingOf(current_level_, id));
    return sibiling ? !sibiling->is_pointed : false;
  }

  // unpoint all sibilings and point the specified sibiling
  void point(const int id) {
    if (canPoint(id)) {
      for (const ItemPtr &sibiling : sibilingsOf(current_level_)) {
        sibiling->is_pointed = false;
      }
      sibilingOf(current_level_, id)->is_pointed = true;
      return;
    }
    throw ros::Exception("Menu::point()");
  }

  // unpointing

  // can unpoint if the specified sibiling is pointed
  bool canUnpoint(const int id) const {
    const ItemConstPtr sibiling(sibilingOf(current_level_, id));
    return sibiling ? sibiling->is_pointed : false;
  }

  // just unpoint the specified sibiling
  void unpoint(const int id) {
    if (canUnpoint(id)) {
      sibilingOf(current_level_, id)->is_pointed = false;
      return;
    }
    throw ros::Exception("Menu::unpoint()");
  }

  // select

  // can select if the specified sibiling has no children and is not selected
  bool canSelect(const int id) const {
    const ItemConstPtr sibiling(sibilingOf(current_level_, id));
    return sibiling ? (sibiling->children.empty() && !sibiling->is_selected) : false;
  }

  // deselect all sibilings and select the specified sibiling.
  // deselect will be skipped if allow_multi_selection is true.
  void select(const int id, const bool allow_multi_selection) {
    if (canSelect(id)) {
      if (!allow_multi_selection) {
        for (const ItemPtr &sibiling : sibilingsOf(current_level_)) {
          sibiling->is_selected = false;
        }
      }
      sibilingOf(current_level_, id)->is_selected = true;
      return;
    }
    throw ros::Exception("Menu::select()");
  }

  // deselect

  // can deselect if the specified sibiling has no children and is selected
  bool canDeselect(const int id) const {
    const ItemConstPtr sibiling(sibilingOf(current_level_, id));
    return sibiling ? (sibiling->children.empty() && sibiling->is_selected) : false;
  }

  // just deselect the specified sibiling
  void deselect(const int id) {
    if (canDeselect(id)) {
      sibilingOf(current_level_, id)->is_selected = false;
      return;
    }
    throw ros::Exception("Menu::deselect()");
  }

  // descending

  // can descend if the specified sibiling has a child
  bool canDescend(const int id) const {
    const ItemConstPtr sibiling(sibilingOf(current_level_, id));
    return sibiling ? !sibiling->children.empty() : false;
  }

  // unpoint and deselect all sibilings, select the specified sibiling
  // and move to the child level of the specified sibiling.
  // deselect will be skipped if allow_multi_selection is true.
  void descend(const int id, const bool allow_multi_selection) {
    if (canDescend(id)) {
      for (const ItemPtr &sibiling : sibilingsOf(current_level_)) {
        sibiling->is_pointed = false;
        if (!allow_multi_selection) {
          sibiling->is_selected = false;
        }
      }
      const ItemPtr sibiling(sibilingOf(current_level_, id));
      sibiling->is_selected = true;
      current_level_ = childLevelOf(sibiling);
      return;
    }
    throw ros::Exception("Menu::descend()");
  }

  // ascending

  // can ascend if the current level is not the first
  bool canAscend() const { return parentOf(current_level_) != root_; }

  // unpoint and deselect all sibilings, deselect and move to the parent
  void ascend() {
    if (canAscend()) {
      for (const ItemPtr &sibiling : sibilingsOf(current_level_)) {
        sibiling->is_pointed = false;
        sibiling->is_selected = false;
      }
      const ItemPtr parent(parentOf(current_level_));
      parent->is_selected = false;
      current_level_ = levelOf(parent);
      return;
    }
    throw ros::Exception("Menu::ascend()");
  }

  // reset

  // unpoint and deselect all items in the menu and move to the first level
  void reset() {
    struct Internal {
      static void reset(const ItemPtr &item) {
        item->is_pointed = false;
        item->is_selected = false;
        for (const ItemPtr &child : item->children) {
          reset(child);
        }
      }
    };

    Internal::reset(root_);
    current_level_ = childLevelOf(root_);
  }

  // exporting

  radial_menu_msgs::StatePtr toState(const ros::Time &stamp, const bool is_enabled) const {
    struct Internal {
      // update State msg by populating state of child menus
      static void updateState(const ItemConstPtr &parent, std::vector< std::string > *const items,
                              std::vector< std::int32_t > *const widths,
                              std::int32_t *const pointed_id,
                              std::vector< std::int32_t > *const selected_ids) {
        const std::size_t n_children(parent->children.size());
        widths->push_back(n_children);
        const std::size_t id_offset(items->size());
        ItemConstPtr next_parent;
        for (std::size_t child_id = 0; child_id < n_children; ++child_id) {
          const ItemConstPtr child(parent->children[child_id]);
          items->push_back(child->title);
          if (child->is_pointed) {
            *pointed_id = child_id + id_offset;
          }
          if (child->is_selected) {
            selected_ids->push_back(child_id + id_offset);
            if (!child->children.empty()) {
              next_parent = child;
            }
          }
        }
        // ascend to a child selected
        if (next_parent) {
          updateState(next_parent, items, widths, pointed_id, selected_ids);
        }
      }
    };

    radial_menu_msgs::StatePtr state(new radial_menu_msgs::State());
    state->header.stamp = stamp;
    state->title = root_->title;
    state->is_enabled = is_enabled;
    state->pointed_id = -1;
    Internal::updateState(root_, &state->items, &state->widths, &state->pointed_id,
                          &state->selected_ids);
    return state;
  }

  std::string toString() const {
    struct Internal {
      static std::string toString(const ItemConstPtr &item, const std::size_t depth = 0) {
        std::string str;
        if (depth <= 0) { // root item
          str += item->title + "\n";
        } else { // non-root item
          str += std::string(depth * 2, ' ') + "[" + (item->is_pointed ? "P" : " ") +
                 (item->is_selected ? "S" : " ") + "] " + item->title + "\n";
        }
        for (const ItemConstPtr &child : item->children) {
          str += toString(child, depth + 1);
        }
        return str;
      }
    };

    return Internal::toString(root_);
  }

  // factory methods

  // Converts XmlRpcValue like:
  //   MenuTitle:
  //     - Item1:
  //       - SubItem1
  //       - SubItem2:
  //         - ...
  //       - SubItem3
  //     - Item2:
  //       - ...
  static MenuPtr fromXmlRpcValue(const XmlRpc::XmlRpcValue &src) {
    struct Internal {
      static ItemPtr itemFromXmlRpcValue(const XmlRpc::XmlRpcValue &src) {
        typedef XmlRpc::XmlRpcValue Src;

        // using const_cast because "XmlRpcValue::begin() const"
        // is not implemented on ROS kinetic :(
        if (src.getType() == Src::TypeStruct && src.size() == 1 &&
            const_cast< Src & >(src).begin()->second.getType() == Src::TypeArray) {
          const Src::ValueStruct::value_type &src_value(*const_cast< Src & >(src).begin());
          ItemPtr item(new Item());
          item->title = src_value.first;
          item->children.resize(src_value.second.size());
          for (int id = 0; id < src_value.second.size(); ++id) {
            ItemPtr &child(item->children[id]);
            child = itemFromXmlRpcValue(src_value.second[id]);
            if (!child) {
              return ItemPtr();
            }
            child->parent = item;
          }
          return item;
        }

        if (src.getType() == Src::TypeString) {
          ItemPtr item(new Item());
          // XmlRpcValue does not have const conversion to std::string &
          // so we have to copy XmlRpcValue before conversion :(
          item->title = static_cast< std::string & >(Src(src));
          return item;
        }

        std::ostringstream oss;
        src.write(oss);
        ROS_ERROR_STREAM("Menu::fromXmlRpcValue(): Error on parsing '" << oss.str()
                                                                       << "': Unexpected format");
        return ItemPtr();
      }
    };

    MenuPtr menu(new Menu());
    menu->root_ = Internal::itemFromXmlRpcValue(src);
    if (!menu->root_) {
      // no message here because already printed in Internal::itemFromXmlRpcValue()
      return MenuPtr();
    }
    menu->current_level_ = childLevelOf(menu->root_);
    if (!menu->current_level_) {
      ROS_ERROR_STREAM("Menu::fromXmlRpcValue(): The menu '" << menu->root_->title
                                                             << "' must have one child at least");
      return MenuPtr();
    }
    return menu;
  }

  static MenuPtr fromParam(const std::string &key) {
    XmlRpc::XmlRpcValue value;
    if (!ros::param::get(key, value)) {
      ROS_ERROR_STREAM("Menu::fromParam(): Cannot get the param '" << key << "'");
      return MenuPtr();
    }
    return fromXmlRpcValue(value);
  }

protected:
  struct Item;
  typedef boost::shared_ptr< Item > ItemPtr;
  typedef boost::shared_ptr< const Item > ItemConstPtr;
  typedef boost::weak_ptr< Item > ItemWeakPtr;

  static ItemPtr parentOf(const ItemConstPtr &item) {
    return item ? item->parent.lock() : ItemPtr();
  }

  static std::vector< ItemPtr > sibilingsOf(const ItemConstPtr &item) {
    const ItemPtr parent(parentOf(item));
    return parent ? parent->children : std::vector< ItemPtr >();
  }

  static ItemPtr sibilingOf(const ItemConstPtr &item, const int id) {
    const std::vector< ItemPtr > sibilings(sibilingsOf(item));
    return (id >= 0 && id < sibilings.size()) ? sibilings[id] : ItemPtr();
  }

  // level (i.e. the first sibiling) of the given item
  static ItemPtr levelOf(const ItemConstPtr &item) { return sibilingOf(item, 0); }

  static ItemPtr childLevelOf(const ItemConstPtr &item) {
    return (item && !item->children.empty()) ? levelOf(item->children.front()) : ItemPtr();
  }

protected:
  struct Item {
    Item() : is_pointed(false), is_selected(false) {}

    std::string title;
    ItemWeakPtr parent;
    std::vector< ItemPtr > children;
    bool is_pointed, is_selected;
  };

  ItemPtr root_, current_level_;
};

} // namespace radial_menu_backend

#endif