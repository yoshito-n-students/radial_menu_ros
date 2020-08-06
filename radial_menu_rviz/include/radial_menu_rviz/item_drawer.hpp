#ifndef RADIAL_MENU_RVIZ_ITEM_DRAWER_HPP
#define RADIAL_MENU_RVIZ_ITEM_DRAWER_HPP

#include <radial_menu_model/item.hpp>
#include <rviz/load_resource.h>

#include <QBitmap>
#include <QPainter>
#include <QRect>
#include <QString>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace radial_menu_rviz {

// forward declarations
class ItemDrawer;
typedef boost::shared_ptr< ItemDrawer > ItemDrawerPtr;
typedef boost::shared_ptr< const ItemDrawer > ItemDrawerConstPtr;
static ItemDrawerPtr createItemDrawer(const radial_menu_model::ItemConstPtr &item);

// abstract drawer
class ItemDrawer {
public:
  virtual void draw(QPainter *const painter, const QRect &rect) const = 0;
};

// *******************************
// Drawer to display the item name
// *******************************

class NameItemDrawer : public ItemDrawer {
  friend ItemDrawerPtr createItemDrawer(const radial_menu_model::ItemConstPtr &);

protected:
  NameItemDrawer(const radial_menu_model::ItemConstPtr &item) : item_(item) {}

public:
  virtual void draw(QPainter *const painter, const QRect &rect) const {
    painter->drawText(rect, Qt::AlignCenter | Qt::TextWordWrap,
                      QString::fromStdString(item_->name()));
  }

protected:
  const radial_menu_model::ItemConstPtr item_;
};

// **************************************
// Drawer to display the alternative text
// **************************************

class AltTxtItemDrawer : public ItemDrawer {
  friend ItemDrawerPtr createItemDrawer(const radial_menu_model::ItemConstPtr &);

protected:
  AltTxtItemDrawer(const radial_menu_model::ItemConstPtr &item) : item_(item) {}

public:
  virtual void draw(QPainter *const painter, const QRect &rect) const {
    painter->drawText(rect, Qt::AlignCenter | Qt::TextWordWrap,
                      QString::fromStdString(item_->altTxt()));
  }

protected:
  const radial_menu_model::ItemConstPtr item_;
};

// ***************************************
// Drawer to display the alternative image
// ***************************************

class ImageItemDrawer : public ItemDrawer {
  friend ItemDrawerPtr createItemDrawer(const radial_menu_model::ItemConstPtr &);

protected:
  ImageItemDrawer(const radial_menu_model::ItemConstPtr &item)
      : bitmap_(rviz::loadPixmap(QString::fromStdString(item->imgURL()), /* fill_cache = */ true)) {
  }

public:
  virtual void draw(QPainter *const painter, const QRect &rect) const {
    painter->drawPixmap(rect, bitmap_);
  }

protected:
  QBitmap bitmap_;
};

// ****************
// Factory function
// ****************

static inline ItemDrawerPtr createItemDrawer(const radial_menu_model::ItemConstPtr &item) {
  switch (item->displayType()) {
  case radial_menu_model::Item::Name:
    return ItemDrawerPtr(new NameItemDrawer(item));
  case radial_menu_model::Item::AltTxt:
    return ItemDrawerPtr(new AltTxtItemDrawer(item));
  case radial_menu_model::Item::Image:
    return ItemDrawerPtr(new ImageItemDrawer(item));
  default:
    ROS_ERROR_STREAM("createImageDrawer(): Unsupported item display type (" +
                     boost::lexical_cast< std::string >(static_cast< int >(item->displayType())) +
                     ")");
    return ItemDrawerPtr();
  }
}

// ****************
// Drawing function
// ****************

static inline bool drawItem(QPainter *const painter, const QRect &rect,
                            const radial_menu_model::ItemConstPtr &item) {
  const ItemDrawerPtr drawer(createItemDrawer(item));
  if (drawer) {
    drawer->draw(painter, rect);
    return true;
  } else {
    return false;
  }
}

} // namespace radial_menu_rviz

#endif