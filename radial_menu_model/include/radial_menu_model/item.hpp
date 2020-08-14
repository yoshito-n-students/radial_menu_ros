#ifndef RADIAL_MENU_MODEL_ITEM_HPP
#define RADIAL_MENU_MODEL_ITEM_HPP

#include <string>
#include <vector>

#include <ros/console.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace radial_menu_model {

// pointer types

class Item;
typedef boost::shared_ptr< Item > ItemPtr;
typedef boost::shared_ptr< const Item > ItemConstPtr;

// menu item.
// state of Item cannot be changed after construction by itemsFromElement()
// because all methods of Item is const.

class Item : public boost::enable_shared_from_this< Item > {
public:
  enum DisplayType { Name, AltTxt, Image };

protected:
  Item() {}

public:
  virtual ~Item() {}

  // propaties

  std::int32_t itemId() const { return item_id_; }

  const std::string &name() const { return name_; }

  std::string path(const char separator = '.') const {
    const ItemConstPtr p(parent());
    return p ? (p->path() + separator + name_) : name_;
  }

  DisplayType displayType() const { return display_type_; }

  const std::string &altTxt() const { return alt_txt_; }

  const std::string &imgURL() const { return img_url_; }

  // root

  ItemConstPtr root() const {
    const ItemConstPtr p(parent());
    return p ? p->root() : shared_from_this();
  }

  // parent

  ItemConstPtr parent() const { return parent_.lock(); }

  ItemConstPtr parentLevel() const {
    const ItemConstPtr p(parent());
    return p ? p->sibiling(0) : ItemConstPtr();
  }

  // sibilings

  int numSibilings() const {
    const ItemConstPtr p(parent());
    return p ? p->children_.size() : 1;
  }

  std::vector< ItemConstPtr > sibilings() const {
    const ItemConstPtr p(parent());
    return p ? p->children_ : std::vector< ItemConstPtr >(1, shared_from_this());
  }

  ItemConstPtr sibiling(const int sid) const {
    const ItemConstPtr p(parent());
    if (p && sid >= 0 && sid < p->children_.size()) {
      return p->children_[sid];
    } else if (!p && sid == 0) {
      return shared_from_this();
    } else {
      return ItemConstPtr();
    }
  }

  ItemConstPtr sibilingLevel() const { return sibiling(0); }

  int depth() const {
    const ItemConstPtr p(parent());
    return p ? p->depth() + 1 : 0;
  }

  // children

  int numChildren() const { return children_.size(); }

  const std::vector< ItemConstPtr > &children() const { return children_; }

  ItemConstPtr child(const int cid) const {
    return (cid >= 0 && cid < children_.size()) ? children_[cid] : ItemConstPtr();
  }

  ItemConstPtr childLevel() const { return child(0); }

  // factory

  static std::vector< ItemConstPtr >
  itemsFromElement(const boost::property_tree::ptree::value_type &elm) {
    namespace bpt = boost::property_tree;

    struct Internal {
      static bool appendItems(const bpt::ptree::value_type &elm,
                              std::vector< ItemConstPtr > *const items,
                              const ItemPtr &parent_item = ItemPtr()) {
        // is the element name "item"?
        if (elm.first != "item") {
          ROS_ERROR_STREAM("Item::itemsFromElement(): Invalid element '" << elm.first << "'");
          return false;
        }

        // create an item and append it to the given list
        const ItemPtr item(new Item());
        item->item_id_ = items->size();
        if (parent_item) {
          item->parent_ = parent_item;
          parent_item->children_.push_back(item);
        }
        items->push_back(item);

        // load the item name from the attribute
        if (!getAttribute(elm, "name", &item->name_)) {
          ROS_ERROR("Item::itemsFromElement(): No attribute 'name'");
          return false;
        }

        // load the display type from the attribute
        const std::string display(attribute(elm, "display", "name"));
        if (display == "name") {
          item->display_type_ = Item::Name;
        } else if (display == "alttxt") {
          item->display_type_ = Item::AltTxt;
          if (!getAttribute(elm, "alttxt", &item->alt_txt_)) {
            ROS_ERROR("Item::itemsFromElement(): No attribute 'alttxt'");
            return false;
          }
        } else if (display == "image") {
          item->display_type_ = Item::Image;
          if (!getAttribute(elm, "imgurl", &item->img_url_)) {
            ROS_ERROR("Item::itemsFromElement(): No attribute 'imgurl'");
            return false;
          }
        } else {
          ROS_ERROR_STREAM("Item::itemsFromElement(): Unknown display type '" << display << "'");
          return false;
        }

        // recursively update the given list
        for (const bpt::ptree::value_type &child_elm : elm.second) {
          if (child_elm.first == "<xmlattr>") {
            continue;
          }
          if (!appendItems(child_elm, items, item)) {
            return false;
          }
        }

        return true;
      }

      // get xml attribute like ros::param::param()
      static std::string attribute(const bpt::ptree::value_type &elm, const std::string &attr,
                                   const std::string &default_val) {
        const boost::optional< std::string > val(
            elm.second.get_optional< std::string >("<xmlattr>." + attr));
        return val ? *val : default_val;
      }

      // get xml attribute like ros::param::getParam()
      static bool getAttribute(const bpt::ptree::value_type &elm, const std::string &attr,
                               std::string *const val) {
        const boost::optional< std::string > opt_val(
            elm.second.get_optional< std::string >("<xmlattr>." + attr));
        if (opt_val) {
          *val = *opt_val;
          return true;
        }
        return false;
      }
    };

    std::vector< ItemConstPtr > items;
    return Internal::appendItems(elm, &items) ? items : std::vector< ItemConstPtr >();
  }

protected:
  typedef boost::weak_ptr< const Item > ItemWeakConstPtr;

  std::int32_t item_id_;
  std::string name_;
  DisplayType display_type_;
  std::string alt_txt_, img_url_;
  ItemWeakConstPtr parent_;
  std::vector< ItemConstPtr > children_;
};
} // namespace radial_menu_model

#endif