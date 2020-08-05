#ifndef RADIAL_MENU_MODEL_ITEM_HPP
#define RADIAL_MENU_MODEL_ITEM_HPP

#include <string>
#include <vector>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace radial_menu_model {

// pointer types

class Item;
typedef boost::shared_ptr< Item > ItemPtr;
typedef boost::shared_ptr< const Item > ItemConstPtr;

// menu item.
// only Model can construct Item because the constructor is protected and Model is the only friend.
// state of Item cannot be changed after construction because all methods of Item is const.

class Item : public boost::enable_shared_from_this< Item > {
  friend class Model;

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
    return p ? p->path() + separator + name_ : name_;
  }

  DisplayType displayType() const { return display_type_; }

  const std::string &altTxt() const { return alt_txt_; }

  const std::string &imageUrl() const { return image_url_; }

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
    return p ? std::vector< ItemConstPtr >(p->children_.begin(), p->children_.end())
             : std::vector< ItemConstPtr >(1, shared_from_this());
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

  std::vector< ItemConstPtr > children() const {
    return std::vector< ItemConstPtr >(children_.begin(), children_.end());
  }

  ItemConstPtr child(const int cid) const {
    return (cid >= 0 && cid < children_.size()) ? children_[cid] : ItemConstPtr();
  }

  ItemConstPtr childLevel() const { return child(0); }

protected:
  typedef boost::weak_ptr< const Item > ItemWeakConstPtr;

  std::int32_t item_id_;
  std::string name_;
  DisplayType display_type_;
  std::string alt_txt_, image_url_;
  ItemWeakConstPtr parent_;
  std::vector< ItemConstPtr > children_;
};
} // namespace radial_menu_model

#endif