#ifndef RADIAL_MENU_RVIZ_HORIZONTAL_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_HORIZONTAL_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cstdint>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>

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
  HorizontalImageDrawer(const radial_menu_msgs::State &state, const HorizontalDrawingProperty &prop)
      : state_(state), prop_(prop) {}

  virtual ~HorizontalImageDrawer() {}

  void setState(const radial_menu_msgs::State &state) { state_ = state; }

  void setProperty(const HorizontalDrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    // plan element layouts on the image
    QSize image_size;
    std::vector< ElementType > element_types;
    std::vector< QRect > bg_rects, text_rects;
    std::vector< QString > texts;
    imageLayout(&image_size, &element_types, &bg_rects, &text_rects, &texts);

    // draw elements on the image
    QImage image(ImageOverlay::formattedImage(image_size, Qt::transparent));
    for (std::size_t i = 0; i < element_types.size(); ++i) {
      switch (element_types[i]) {
      case TitleElement:
        drawBackground(&image, prop_.title_bg_rgb, bg_rects[i]);
        drawText(&image, prop_.title_rgb, text_rects[i], texts[i]);
        break;
      case SelectedElement:
        drawBackground(&image, prop_.item_bg_rgb_selected, bg_rects[i]);
        drawText(&image, prop_.item_rgb_selected, text_rects[i], texts[i]);
        break;
      case PointedElement:
        drawBackground(&image, prop_.item_bg_rgb_pointed, bg_rects[i]);
        drawText(&image, prop_.item_rgb_pointed, text_rects[i], texts[i]);
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

  // drawing functions

  void drawBackground(QImage *const image, const QRgb &rgb, const QRect &rect) const {
    // painter for the given image
    QPainter painter(image);
    painter.setRenderHint(QPainter::Antialiasing);

    // draw background
    const QColor color(makeColor(rgb, prop_.bg_alpha));
    painter.setPen(color);
    painter.setBrush(color);
    painter.drawRect(rect);
  }

  void drawText(QImage *const image, const QRgb &rgb, const QRect &rect,
                const QString &text) const {
    // painter for the given image
    QPainter painter(image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);

    // draw text
    painter.setPen(makeColor(rgb, prop_.text_alpha));
    painter.drawText(rect, Qt::AlignCenter, text);
  }

  // helper functions

  // calc image size and element layouts
  void imageLayout(QSize *const image_size, std::vector< ElementType > *const element_types,
                   std::vector< QRect > *const bg_rects, std::vector< QRect > *const text_rects,
                   std::vector< QString > *const texts) const {
    element_types->clear();
    bg_rects->clear();
    text_rects->clear();
    texts->clear();

    //
    // 1. enumerate drawable elements
    //

    // evaluate the title
    {
      const QString text(QString::fromStdString(state_.title));
      QRect bg_rect, text_rect;
      elementLayout(text, &bg_rect, &text_rect);
      if (bg_rect.isValid() && text_rect.isValid()) {
        element_types->push_back(TitleElement);
        bg_rects->push_back(bg_rect);
        text_rects->push_back(text_rect);
        texts->push_back(text);
      }
    }

    // evaluate the selected items
    for (const std::int32_t id : state_.selected_ids) {
      if (id >= 0 && id < state_.items.size()) {
        const QString text(QString::fromStdString(state_.items[id]));
        QRect bg_rect, text_rect;
        elementLayout(text, &bg_rect, &text_rect);
        if (bg_rect.isValid() && text_rect.isValid()) {
          element_types->push_back(SelectedElement);
          bg_rects->push_back(bg_rect);
          text_rects->push_back(text_rect);
          texts->push_back(text);
        }
      }
    }

    // evaluate the pointed item
    if (state_.pointed_id >= 0 && state_.pointed_id < state_.items.size()) {
      const QString text(QString::fromStdString(state_.items[state_.pointed_id]));
      QRect bg_rect, text_rect;
      elementLayout(text, &bg_rect, &text_rect);
      if (bg_rect.isValid() && text_rect.isValid()) {
        element_types->push_back(PointedElement);
        bg_rects->push_back(bg_rect);
        text_rects->push_back(text_rect);
        texts->push_back(text);
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

    // layout of text elements
    for (std::size_t i = 0; i < bg_rects->size(); ++i) {
      (*text_rects)[i].moveCenter((*bg_rects)[i].center());
    }

    //
    // 3. image size
    //

    image_size->setWidth(bg_rects->empty() ? 0 : bg_rects->back().right());
    image_size->setHeight(bg_rects->empty() ? 0 : bg_rects->back().bottom());
  }

  // calc bounding rectangles of element's background and text.
  // the top left corner of the background rectangle will be aligined to (0, 0).
  // if the text is not drawable, returns null rectangles.
  void elementLayout(const QString &text, QRect *const bg_rect, QRect *const text_rect) const {
    if (!text.isEmpty()) {
      *text_rect = QFontMetrics(prop_.font).boundingRect(QRect(), Qt::AlignCenter, text);
      *bg_rect = *text_rect +
                 QMargins(prop_.bg_padding, prop_.bg_padding, prop_.bg_padding, prop_.bg_padding);
      const QPoint offset(-bg_rect->topLeft());
      text_rect->translate(offset);
      bg_rect->translate(offset);
    } else {
      *bg_rect = QRect();
      *text_rect = QRect();
    }
  }

  static QColor makeColor(const QRgb &rgb, const int alpha) {
    QColor color;
    color.setRgb(rgb);
    color.setAlpha(alpha);
    return color;
  }

protected:
  radial_menu_msgs::State state_;
  HorizontalDrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif