#ifndef RADIAL_MENU_BACKEND_MENU_HPP
#define RADIAL_MENU_BACKEND_MENU_HPP

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace radial_menu_backend {

class Menu;
typedef boost::shared_ptr< Menu > MenuPtr;
typedef boost::shared_ptr< const Menu > MenuConstPtr;

// Minimal implementation of a radial menu data.
// A menu must be created via fromXmlRpcValue().
// Navigation must start from the root menu
// and performed by sibiling(), descend() or ascend().
class Menu : public boost::enable_shared_from_this< Menu > {
protected:
  Menu() : is_selected_(false), is_pointed_(false) {}

public:
  virtual ~Menu() {}

  // horizontal navigation

  std::size_t numSibilings() const { return parent_ ? parent_->children_.size() : 1; }

  MenuConstPtr sibiling(const std::size_t id) const {
    if (parent_ && id >= 0 && id < parent_->children_.size()) {
      return parent_->children_[id];
    } else if (!parent_ && id == 0) {
      return shared_from_this();
    }
    throw std::logic_error("Menu::sibiling() const: Invalid id (" +
                           boost::lexical_cast< std::string >(id) + ")");
  }

  MenuPtr sibiling(const std::size_t id) {
    if (parent_ && id >= 0 && id < parent_->children_.size()) {
      return parent_->children_[id];
    } else if (!parent_ && id == 0) {
      return shared_from_this();
    }
    throw std::logic_error("Menu::sibiling(): Invalid id (" +
                           boost::lexical_cast< std::string >(id) + ")");
  }

  // pointing

  // unpoint all sibilings and then point this
  void point() {
    for (std::size_t id = 0; id < numSibilings(); ++id) {
      sibiling(id)->is_pointed_ = false;
    }
    is_pointed_ = true;
  }

  // select

  bool canSelect() const { return !is_selected_ && children_.empty(); }

  // deselect all sibilings, if multi-selection is not allowed, and then select this
  void select(const bool allow_multi_selection) {
    if (canSelect()) {
      if (!allow_multi_selection) {
        for (std::size_t id = 0; id < numSibilings(); ++id) {
          sibiling(id)->is_selected_ = false;
        }
      }
      is_selected_ = true;
      return;
    }
    throw std::logic_error("Menu::select(): Cannot select '" + title_ + "'");
  }

  // deselect

  bool canDeselect() const { return is_selected_ && children_.empty(); }

  void deselect() {
    if (canDeselect()) {
      is_selected_ = false;
      return;
    }
    throw std::logic_error("Menu::deselect(): Cannot deselect '" + title_ + "'");
  }

  // descending

  bool canDescend() const { return !children_.empty(); }

  // deselect all sibilings, unpoint this, select this
  MenuPtr descend() {
    if (canDescend()) {
      for (std::size_t id = 0; id < numSibilings(); ++id) {
        sibiling(id)->is_selected_ = false;
      }
      is_pointed_ = false;
      is_selected_ = true;
      return children_.front();
    }
    throw std::logic_error("Menu::descend(): Cannot descend from '" + title_ + "'");
  }

  // ascending

  bool isRoot() const { return !parent_; }

  bool canAscend() const { return !isRoot(); }

  // deselect all sibilings and parent, unpoint this, unselect this
  MenuPtr ascend() {
    if (canAscend()) {
      for (std::size_t id = 0; id < numSibilings(); ++id) {
        sibiling(id)->is_selected_ = false;
      }
      parent_->is_selected_ = false;
      is_pointed_ = false;
      is_selected_ = false;
      return parent_;
    }
    throw std::logic_error("Menu::ascend(): Cannot ascend from '" + title_ + "'");
  }

  // reset

  // ascend to the root, deselect and unpoint all items on the menu
  MenuPtr reset() {
    struct Impl {
      static void reset(const MenuPtr &menu) {
        menu->is_pointed_ = false;
        menu->is_selected_ = false;
        for (const MenuPtr &child : menu->children_) {
          reset(child);
        }
      }
    };

    const MenuPtr r(root());
    Impl::reset(r);
    return r;
  }

  void unpointAll() {
    struct Impl {
      static void unpointAll(const MenuPtr &menu) {
        menu->is_pointed_ = false;
        for (const MenuPtr &child : menu->children_) {
          unpointAll(child);
        }
      }
    };

    Impl::unpointAll(root());
  }

  // type conversion

  radial_menu_msgs::StatePtr toState(const ros::Time &stamp, const bool is_enabled) const {
    struct Impl {
      static radial_menu_msgs::StatePtr toState(const MenuConstPtr &root, const ros::Time &stamp,
                                                const bool is_enabled) {
        radial_menu_msgs::StatePtr state(new radial_menu_msgs::State());
        state->header.stamp = stamp;
        state->title = root->title_;
        state->is_enabled = is_enabled;
        state->pointed_id = -1;
        updateState(root, &state->items, &state->widths, &state->pointed_id, &state->selected_ids);
        return state;
      }

      // update State msg by populating state of child menus
      static void updateState(const MenuConstPtr &parent, std::vector< std::string > *const items,
                              std::vector< std::int32_t > *const widths,
                              std::int32_t *const pointed_id,
                              std::vector< std::int32_t > *const selected_ids) {
        // terminate if the parent menu has no children
        const std::size_t n_children(parent->children_.size());
        if (n_children <= 0) {
          return;
        }
        // update states
        widths->push_back(n_children);
        const std::size_t id_offset(items->size());
        MenuConstPtr next_parent;
        for (std::size_t child_id = 0; child_id < n_children; ++child_id) {
          const MenuConstPtr child(parent->children_[child_id]);
          items->push_back(child->title_);
          if (child->is_pointed_) {
            *pointed_id = child_id + id_offset;
          }
          if (child->is_selected_) {
            selected_ids->push_back(child_id + id_offset);
            next_parent = child;
          }
        }
        // ascend to a child selected
        if (next_parent) {
          updateState(next_parent, items, widths, pointed_id, selected_ids);
        }
      }
    };

    return Impl::toState(root(), stamp, is_enabled);
  }

  std::string toString() const {
    struct Impl {
      static std::string toString(const MenuConstPtr &menu, const std::size_t n_indent) {
        std::string str;
        str += std::string(n_indent, ' ') + "[" + (menu->is_pointed_ ? "P" : " ") +
               (menu->is_selected_ ? "S" : " ") + "] " + menu->title_ + "\n";
        for (const MenuConstPtr &child : menu->children_) {
          str += toString(child, n_indent + 2);
        }
        return str;
      }
    };

    return Impl::toString(root(), 0);
  }

protected:
  // internal use

  MenuConstPtr root() const { return parent_ ? parent_->root() : shared_from_this(); }

  MenuPtr root() { return parent_ ? parent_->root() : shared_from_this(); }

public:
  // factory methods

  // Converts XmlRpcValue from rosparam like:
  //   "MLB":
  //     - "American":
  //       - "East":
  //         - "Baltimore"
  //         - "Boston"
  //         - ...
  //       - "Central":
  //         - "Chicago"
  //         - ...
  //       - "West":
  //         - ....
  //     - "National":
  //       - ...
  static MenuPtr fromXmlRpcValue(const XmlRpc::XmlRpcValue &src) {
    typedef XmlRpc::XmlRpcValue Src;

    if (src.getType() == Src::TypeStruct && src.size() == 1 &&
        src.begin()->second.getType() == Src::TypeArray) {
      const Src::ValueStruct::value_type &src_value(*src.begin());
      MenuPtr menu(new Menu());
      menu->title_ = src_value.first;
      menu->children_.resize(src_value.second.size());
      for (int i = 0; i < src_value.second.size(); ++i) {
        MenuPtr &child(menu->children_[i]);
        child = fromXmlRpcValue(src_value.second[i]);
        if (!child) {
          return MenuPtr();
        }
        child->parent_ = menu;
      }
      return menu;
    }

    if (src.getType() == Src::TypeString) {
      MenuPtr menu(new Menu());
      // XmlRpcValue does not have const conversion to std::string
      // so we have to copy XmlRpcValue before conversion :(
      menu->title_ = static_cast< std::string >(Src(src));
      return menu;
    }

    std::ostringstream oss;
    src.write(oss);
    ROS_ERROR_STREAM("Menu::fromXmlRpcValue(): Error on parsing '" << oss.str()
                                                                   << "': Unexpected format");
    return MenuPtr();
  }

  static MenuPtr fromParam(const std::string &key) {
    XmlRpc::XmlRpcValue value;
    return ros::param::get(key, value) ? fromXmlRpcValue(value) : MenuPtr();
  }

protected:
  std::string title_;
  MenuPtr parent_;
  std::vector< MenuPtr > children_;

  bool is_selected_, is_pointed_;
};
} // namespace radial_menu_backend

#endif