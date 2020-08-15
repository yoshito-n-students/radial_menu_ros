#ifndef RADIAL_MENU_MODEL_ITEM_HPP
#define RADIAL_MENU_MODEL_ITEM_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <radial_menu_model/xml_element.hpp>
#include <ros/console.h>

namespace radial_menu_model {

// pointer types

class Item;
typedef std::shared_ptr< Item > ItemPtr;
typedef std::shared_ptr< const Item > ItemConstPtr;

// menu item.
// state of Item cannot be changed after construction by itemsFromDescription()
// because all methods of Item is const.

class Item : public std::enable_shared_from_this< Item > {
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
    struct Internal {
      static bool appendItems(const XmlElement &elm, std::vector< ItemConstPtr > *const items,
                              const ItemPtr &parent_item = ItemPtr(), const int default_row = 0) {
        // is the element name "item"?
        if (elm.name() != "item") {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Unexpected element '" << elm.name()
                                                                                << "'");
          return false;
        }

        // create an item and append it to the given list
        const ItemPtr item(new Item());
        item->item_id_ = items->size();
        items->push_back(item);

        // associate the item with the parent
        if (parent_item) {
          const int row(elm.attribute("row", default_row));
          if (row < 0 || row >= parent_item->children_.size()) {
            ROS_ERROR_STREAM("Item::itemsFromDescription(): '" << row << "' is out of row range");
            return false;
          }
          if (parent_item->children_[row]) {
            ROS_ERROR_STREAM("Item::itemsFromDescription(): Multiple items in the row '" << row
                                                                                         << "'");
            return false;
          }
          parent_item->children_[row] = item;
          item->parent_ = parent_item;
        }

        // load the item name from the attribute
        if (!elm.getAttribute("name", &item->name_)) {
          ROS_ERROR("Item::itemsFromDescription(): No attribute 'name'");
          return false;
        }

        // load the display type from the attribute
        const std::string display(elm.attribute< std::string >("display", "name"));
        if (display == "name") {
          item->display_type_ = Item::Name;
        } else if (display == "alttxt") {
          item->display_type_ = Item::AltTxt;
          if (!elm.getAttribute("alttxt", &item->alt_txt_)) {
            ROS_ERROR("Item::itemsFromDescription(): No attribute 'alttxt'");
            return false;
          }
        } else if (display == "image") {
          item->display_type_ = Item::Image;
          if (!elm.getAttribute("imgurl", &item->img_url_)) {
            ROS_ERROR("Item::itemsFromDescription(): No attribute 'imgurl'");
            return false;
          }
        } else {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Unknown display type '" << display
                                                                                  << "'");
          return false;
        }

        // allocate child items
        const int rows(elm.attribute("rows", elm.numChildElements()));
        if (rows < 0) {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Invalid row size '" << rows << "'");
          return false;
        }
        item->children_.resize(rows);

        // recursively update the given list
        const std::vector< XmlElementConstPtr > child_elms(elm.childElements());
        for (std::size_t i = 0; i < child_elms.size(); ++i) {
          if (!appendItems(*child_elms[i], items, item, i)) {
            return false;
          }
        }

        return true;
      }
    };

    // parse the given xml description
    const XmlElementConstPtr elm(XmlElement::fromString(desc));
    if (!elm) {
      return std::vector< ItemConstPtr >();
    }

    // populate items by parsing the root xml element
    std::vector< ItemConstPtr > items;
    if (!Internal::appendItems(*elm, &items)) {
      return std::vector< ItemConstPtr >();
    }

    return items;
  }

protected:
  typedef std::weak_ptr< const Item > ItemWeakConstPtr;

  std::int32_t item_id_; // int32_t is the type of ids in State msg
  std::string name_;
  DisplayType display_type_;
  std::string alt_txt_, img_url_;
  ItemWeakConstPtr parent_;
  std::vector< ItemConstPtr > children_;
};
} // namespace radial_menu_model

#endif