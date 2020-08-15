#ifndef RADIAL_MENU_MODEL_ITEM_HPP
#define RADIAL_MENU_MODEL_ITEM_HPP

#include <sstream>
#include <string>
#include <vector>

#include <ros/console.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace radial_menu_model {

// pointer types

class Item;
typedef boost::shared_ptr< Item > ItemPtr;
typedef boost::shared_ptr< const Item > ItemConstPtr;

// menu item.
// state of Item cannot be changed after construction by itemsFromDescription()
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
    return p ? p->sibilingLevel() : ItemConstPtr();
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

  ItemConstPtr sibilingLevel() const {
    const ItemConstPtr p(parent());
    if (p) {
      for (const ItemConstPtr &s : p->children_) {
        if (s) {
          return s;
        }
      }
    }
    return shared_from_this();
  }

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

  ItemConstPtr childLevel() const {
    for (const ItemConstPtr &c : children_) {
      if (c) {
        return c;
      }
    }
    return ItemConstPtr();
  }

  // factory

  static std::vector< ItemConstPtr > itemsFromDescription(const std::string &desc) {
    namespace bpt = boost::property_tree;

    // parse the given xml description
    bpt::ptree xml;
    try {
      std::istringstream iss(desc);
      bpt::read_xml(iss, xml, bpt::xml_parser::no_comments);
    } catch (const bpt::ptree_error &ex) {
      ROS_ERROR_STREAM("Item::itemsFromDescription(): " << ex.what());
      return std::vector< ItemConstPtr >();
    }

    // find the root xml element
    if (xml.size() != 1) {
      ROS_ERROR("Item::itemsFromDescription(): Non unique root element in xml");
      return std::vector< ItemConstPtr >();
    }

    // populate items by parsing the root xml element
    std::vector< ItemConstPtr > items;
    if (!appendItems(xml.front(), &items)) {
      return std::vector< ItemConstPtr >();
    }

    return items;
  }

protected:
  // factory helpers

  typedef boost::property_tree::ptree::value_type XmlElement;

  static bool appendItems(const XmlElement &elm, std::vector< ItemConstPtr > *const items,
                          const ItemPtr &parent_item = ItemPtr(), const int default_row = 0) {
    // is the element name "item"?
    if (elm.first != "item") {
      ROS_ERROR_STREAM("Item::appendItems(): Invalid element '" << elm.first << "'");
      return false;
    }

    // create an item and append it to the given list
    const ItemPtr item(new Item());
    item->item_id_ = items->size();
    items->push_back(item);

    // associate the item with the parent
    if (parent_item) {
      const int row(attribute(elm, "row", default_row));
      if (row < 0 || row >= parent_item->children_.size()) {
        ROS_ERROR_STREAM("Item::appendItems(): Invalid row '" << row << "'");
        return false;
      }
      if (parent_item->children_[row]) {
        ROS_ERROR_STREAM("Item::appendItems(): Multiple items in the row '" << row << "'");
        return false;
      }
      parent_item->children_[row] = item;
      item->parent_ = parent_item;
    }

    // load the item name from the attribute
    if (!getAttribute(elm, "name", &item->name_)) {
      ROS_ERROR("Item::appendItems(): No attribute 'name'");
      return false;
    }

    // load the display type from the attribute
    const std::string display(attribute< std::string >(elm, "display", "name"));
    if (display == "name") {
      item->display_type_ = Item::Name;
    } else if (display == "alttxt") {
      item->display_type_ = Item::AltTxt;
      if (!getAttribute(elm, "alttxt", &item->alt_txt_)) {
        ROS_ERROR("Item::appendItems(): No attribute 'alttxt'");
        return false;
      }
    } else if (display == "image") {
      item->display_type_ = Item::Image;
      if (!getAttribute(elm, "imgurl", &item->img_url_)) {
        ROS_ERROR("Item::appendItems(): No attribute 'imgurl'");
        return false;
      }
    } else {
      ROS_ERROR_STREAM("Item::appendItems(): Unknown display type '" << display << "'");
      return false;
    }

    // allocate child items
    const int rows(attribute(elm, "rows", numChildElements(elm)));
    if (rows < 0) {
      ROS_ERROR_STREAM("Item::appendItems(): Invalid row size '" << rows << "'");
      return false;
    }
    item->children_.resize(rows);

    // recursively update the given list
    int default_child_row(0);
    for (const XmlElement &child_elm : elm.second) {
      if (child_elm.first == "<xmlattr>") {
        continue;
      }
      if (!appendItems(child_elm, items, item, default_child_row)) {
        return false;
      }
      ++default_child_row;
    }

    return true;
  }

  static int numChildElements(const XmlElement &elm) {
    int n_child_elm(0);
    for (const XmlElement &child_elm : elm.second) {
      if (child_elm.first != "<xmlattr>") {
        ++n_child_elm;
      }
    }
    return n_child_elm;
  }

  // get xml attribute like ros::param::param()
  template < typename T >
  static T attribute(const XmlElement &elm, const std::string &attr, const T &default_val) {
    const boost::optional< T > val(elm.second.get_optional< T >("<xmlattr>." + attr));
    return val ? *val : default_val;
  }

  // get xml attribute like ros::param::getParam()
  template < typename T >
  static bool getAttribute(const XmlElement &elm, const std::string &attr, T *const val) {
    const boost::optional< T > opt_val(elm.second.get_optional< T >("<xmlattr>." + attr));
    if (opt_val) {
      *val = *opt_val;
      return true;
    }
    return false;
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