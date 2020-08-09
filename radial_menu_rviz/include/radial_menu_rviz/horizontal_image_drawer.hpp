#ifndef RADIAL_MENU_RVIZ_HORIZONTAL_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_HORIZONTAL_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cstdint>
#include <vector>

#include <radial_menu_model/model.hpp>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>
#include <rviz/load_resource.h>

#include <QBrush>
#include <QColor>
#include <QFontMetrics>
#include <QImage>
#include <QMargins>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QRect>
#include <QRgb>
#include <QSize>
#include <QString>

namespace radial_menu_rviz {

class HorizontalImageDrawer {
public:
  HorizontalImageDrawer(const radial_menu_model::ModelConstPtr &model,
                        const HorizontalDrawingProperty &prop) {
    setModel(model);
    setProperty(prop);
  }

  virtual ~HorizontalImageDrawer() {}

  void setModel(const radial_menu_model::ModelConstPtr &model) { model_ = model; }

  void setProperty(const HorizontalDrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    // plan element layouts on the image
    QSize image_size;
    std::vector< ElementType > element_types;
    std::vector< QRect > bg_rects, fg_rects;
    std::vector< radial_menu_model::ItemConstPtr > items;
    imageLayout(&image_size, &element_types, &bg_rects, &fg_rects, &items);

    // draw elements on the image
    QImage image(ImageOverlay::formattedImage(image_size, Qt::transparent));
    QPainter painter(&image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setRenderHint(QPainter::Antialiasing);
    for (std::size_t i = 0; i < element_types.size(); ++i) {
      switch (element_types[i]) {
      case TitleElement:
        drawBackground(&painter, prop_.title_bg_rgb, bg_rects[i]);
        drawForeground(&painter, prop_.title_rgb, fg_rects[i], items[i]);
        break;
      case SelectedElement:
        drawBackground(&painter, prop_.item_bg_rgb_selected, bg_rects[i]);
        drawForeground(&painter, prop_.item_rgb_selected, fg_rects[i], items[i]);
        break;
      case PointedElement:
        drawBackground(&painter, prop_.item_bg_rgb_pointed, bg_rects[i]);
        drawForeground(&painter, prop_.item_rgb_pointed, fg_rects[i], items[i]);
        break;
      default:
        ROS_ERROR_STREAM("HorizontalImageDrawer::draw(): unexpected element type ("
                         << static_cast< int >(element_types[i]) << "). Will not draw.");
        break;
      }
    }

    return image;
  }

protected:
  enum ElementType { TitleElement, SelectedElement, PointedElement };

  // layout functions

  // calc image size and element layouts
  void imageLayout(QSize *const image_size, std::vector< ElementType > *const element_types,
                   std::vector< QRect > *const bg_rects, std::vector< QRect > *const fg_rects,
                   std::vector< radial_menu_model::ItemConstPtr > *const items) const {
    element_types->clear();
    bg_rects->clear();
    fg_rects->clear();
    items->clear();

    //
    // 1. enumerate drawable elements
    //

    // evaluate the title element
    {
      const radial_menu_model::ItemConstPtr item(model_->root());
      QRect bg_rect, fg_rect;
      elementLayout(item, &bg_rect, &fg_rect);
      if (bg_rect.isValid() && fg_rect.isValid()) {
        element_types->push_back(TitleElement);
        bg_rects->push_back(bg_rect);
        fg_rects->push_back(fg_rect);
        items->push_back(item);
      }
    }

    // evaluate the selected elements
    for (const radial_menu_model::ItemConstPtr &item : model_->selected()) {
      QRect bg_rect, fg_rect;
      elementLayout(item, &bg_rect, &fg_rect);
      if (bg_rect.isValid() && fg_rect.isValid()) {
        element_types->push_back(SelectedElement);
        bg_rects->push_back(bg_rect);
        fg_rects->push_back(fg_rect);
        items->push_back(item);
      }
    }

    // evaluate the pointed element
    {
      const radial_menu_model::ItemConstPtr item(model_->pointed());
      if (item) {
        QRect bg_rect, fg_rect;
        elementLayout(item, &bg_rect, &fg_rect);
        if (bg_rect.isValid() && fg_rect.isValid()) {
          element_types->push_back(PointedElement);
          bg_rects->push_back(bg_rect);
          fg_rects->push_back(fg_rect);
          items->push_back(item);
        }
      }
    }

    //
    // 2. layout elements
    //

    // vertical layout of background elements
    {
      QRect united_rect;
      for (const QRect &bg_rect : *bg_rects) {
        united_rect |= bg_rect;
      }
      for (QRect &bg_rect : *bg_rects) {
        bg_rect.setHeight(united_rect.height());
      }
    }

    // horizontal layout of background elements
    for (std::size_t i = 1; i < bg_rects->size(); ++i) {
      (*bg_rects)[i].moveLeft((*bg_rects)[i - 1].right() + prop_.line_width);
    }

    // layout of foreground elements
    for (std::size_t i = 0; i < bg_rects->size(); ++i) {
      (*fg_rects)[i].moveCenter((*bg_rects)[i].center());
    }

    //
    // 3. image size
    //

    image_size->setWidth(bg_rects->empty() ? 0 : bg_rects->back().right());
    image_size->setHeight(bg_rects->empty() ? 0 : bg_rects->back().bottom());
  }

  // calc bounding rectangles of element's background and foreground.
  // the top left corner of the background rectangle will be aligined to (0, 0).
  // if the text is not drawable, returns null rectangles.
  void elementLayout(const radial_menu_model::ItemConstPtr &item, QRect *const bg_rect,
                     QRect *const fg_rect) const {
    // set foreground rect
    switch (item->displayType()) {
    case radial_menu_model::Item::Name:
      fg_rect->setHeight(prop_.fg_height);
      fg_rect->setWidth(textWidth(prop_.font, QString::fromStdString(item->name())));
      break;
    case radial_menu_model::Item::AltTxt:
      fg_rect->setHeight(prop_.fg_height);
      fg_rect->setWidth(textWidth(prop_.font, QString::fromStdString(item->altTxt())));
      break;
    case radial_menu_model::Item::Image:
      fg_rect->setHeight(prop_.fg_height);
      fg_rect->setWidth(prop_.fg_height);
      break;
    default:
      ROS_ERROR_STREAM("HorizontalImageDrawer::elementLayout(): the item '"
                       << item->name() << "' has unexpected type ("
                       << static_cast< int >(item->displayType()) << ")");
      fg_rect->setHeight(1);
      fg_rect->setWidth(1);
      break;
    }

    // set background rect
    *bg_rect = *fg_rect + uniformMargins(prop_.bg_padding);

    // align rects
    bg_rect->translate(-bg_rect->topLeft());
    fg_rect->moveCenter(bg_rect->center());
  }

  static int textWidth(const QFont &font, const QString &text) {
    return QFontMetrics(font).boundingRect(QRect(), Qt::AlignCenter, text).width();
  }

  static QMargins uniformMargins(const int margin) {
    return QMargins(margin, margin, margin, margin);
  }

  // drawing functions

  void drawBackground(QPainter *const painter, const QRgb &rgb, const QRect &rect) const {
    const QColor color(makeColor(rgb, prop_.bg_alpha));
    painter->setPen(color);
    painter->setBrush(color);
    painter->drawRect(rect);
  }

  void drawForeground(QPainter *const painter, const QRgb &rgb, const QRect &rect,
                      const radial_menu_model::ItemConstPtr &item) const {
    painter->setPen(makeColor(rgb, prop_.text_alpha));
    switch (item->displayType()) {
    case radial_menu_model::Item::Name:
      painter->drawText(rect, Qt::AlignCenter, QString::fromStdString(item->name()));
      break;
    case radial_menu_model::Item::AltTxt:
      painter->drawText(rect, Qt::AlignCenter, QString::fromStdString(item->altTxt()));
      break;
    case radial_menu_model::Item::Image:
      painter->drawPixmap(
          rect, rviz::loadPixmap(QString::fromStdString(item->imgURL()), /* fill_cache = */ true));
      break;
    default:
      ROS_ERROR_STREAM("HorizontalImageDrawer::drawForeground(): the item '"
                       << item->name() << "' has unexpected type ("
                       << static_cast< int >(item->displayType()) << ")");
      break;
    }
  }

  static QColor makeColor(const QRgb &rgb, const int alpha) {
    QColor color;
    color.setRgb(rgb);
    color.setAlpha(alpha);
    return color;
  }

protected:
  radial_menu_model::ModelConstPtr model_;
  HorizontalDrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif