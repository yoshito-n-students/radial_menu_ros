#ifndef RADIAL_MENU_RVIZ_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>

#include <QBrush>
#include <QColor>
#include <QFontMetrics>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QRect>
#include <QRgb>
#include <QSize>
#include <QString>

namespace radial_menu_rviz {

class ImageDrawer {
public:
  ImageDrawer(const radial_menu_msgs::State &state, const DrawingProperty &prop)
      : state_(state), prop_(prop) {}

  virtual ~ImageDrawer() {}

  void setState(const radial_menu_msgs::State &state) { state_ = state; }

  void setProperty(const DrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    switch (state_.state) {
    case radial_menu_msgs::State::STATE_OPENED: {
      QImage image(ImageOverlay::formattedImage(imageSize(), Qt::transparent));
      drawBackground(&image);
      drawTexts(&image);
      return image;
    }
    default:
      ROS_ERROR_STREAM("ImageDrawer::draw(): unexpected menu state ("
                       << state_.state << "). Will fallback to closed state.");
    case radial_menu_msgs::State::STATE_CLOSED:
      return QImage();
    }
  }

protected:
  enum ItemType { DefaultItem, PointedItem, SelectedItem, InvalidItem };

  // drawing functions

  void drawBackground(QImage *const image) const {
    // painter for RGB channel of the given image
    QPainter rgb_painter(image);
    rgb_painter.setRenderHint(QPainter::Antialiasing);

    // painter for alpha channel of the given image
    QImage alpha_image(QImage(image->size(), QImage::Format_Grayscale8));
    alpha_image.fill(QColor(0, 0, 0)); // defaultly transparent
    QPainter alpha_painter(&alpha_image);
    alpha_painter.setRenderHint(QPainter::Antialiasing);

    // common properties
    const QRect image_rect(image->rect());
    const QPoint image_center(image_rect.center());

    // draw item areas by pies
    if (!state_.items.empty()) {
      // common properties of pies
      const QPen pen_default(prop_.item_bg_rgb_default), pen_pointed(prop_.item_bg_rgb_pointed),
          pen_selected(prop_.item_bg_rgb_selected);
      const QBrush brush_default(prop_.item_bg_rgb_default),
          brush_pointed(prop_.item_bg_rgb_pointed), brush_selected(prop_.item_bg_rgb_selected);
      const int span_angle(itemSpanAngle());
      // set tools for alpha painter
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter.setPen(bg_alpha);
      alpha_painter.setBrush(bg_alpha);
      // draw each pie
      for (int i = 0; i < state_.items.size(); ++i) {
        // set tools for rgb painter according to item type
        const int item_type(itemType(i));
        switch (item_type) {
        case PointedItem:
          rgb_painter.setPen(pen_pointed);
          rgb_painter.setBrush(brush_pointed);
          break;
        case SelectedItem:
          rgb_painter.setPen(pen_selected);
          rgb_painter.setBrush(brush_selected);
          break;
        default:
          ROS_ERROR_STREAM("ImageDrawer::drawBackground(): unexpected item type ("
                           << item_type << "). Will fallback to the default.");
        case DefaultItem:
          rgb_painter.setPen(pen_default);
          rgb_painter.setBrush(brush_default);
          break;
        }
        // draw a pie
        const int center_angle(itemCenterAngle(i));
        const int start_angle(center_angle - span_angle / 2);
        rgb_painter.drawPie(image_rect, start_angle, span_angle);
        alpha_painter.drawPie(image_rect, start_angle, span_angle);
      }
    }

    // draw transparent lines between item areas
    if (!state_.items.empty() && prop_.line_width > 0) {
      alpha_painter.setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      for (int i = 0; i < state_.items.size(); ++i) {
        alpha_painter.drawLine(image_center, image_center + relativeItemLineEnd(i));
      }
    }

    // draw the title area by a filled circle
    if (prop_.title_area_radius > 0) {
      //
      rgb_painter.setPen(QPen(prop_.title_bg_rgb));
      rgb_painter.setBrush(QBrush(prop_.title_bg_rgb));
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter.setPen(bg_alpha);
      alpha_painter.setBrush(bg_alpha);
      //
      const QPoint relative_corner(prop_.title_area_radius, prop_.title_area_radius);
      const QRect rect(image_center - relative_corner, image_center + relative_corner);
      rgb_painter.drawEllipse(rect);
      alpha_painter.drawEllipse(rect);
    }

    // draw transparent lines between title and item areas
    if (prop_.line_width > 0) {
      alpha_painter.setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      alpha_painter.setBrush(Qt::NoBrush);
      const QPoint relative_corner(prop_.title_area_radius + prop_.line_width / 2,
                                   prop_.title_area_radius + prop_.line_width / 2);
      alpha_painter.drawEllipse(
          QRect(image_center - relative_corner, image_center + relative_corner));
    }

    // apply the alpha channel to the given image
    // (need to release the images from the painters in advance)
    rgb_painter.end();
    alpha_painter.end();
    image->setAlphaChannel(alpha_image);
  }

  void drawTexts(QImage *const image) const {
    QPainter painter(image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);

    const QFontMetrics font_metrics(prop_.font);
    const QPoint image_center(image->rect().center());

    // draw item texts
    if (!state_.items.empty()) {
      const QPen pen_default(makeColor(prop_.item_rgb_default, prop_.text_alpha)),
          pen_pointed(makeColor(prop_.item_rgb_pointed, prop_.text_alpha)),
          pen_selected(makeColor(prop_.item_rgb_selected, prop_.text_alpha));
      for (int i = 0; i < state_.items.size(); ++i) {
        // set tools for the painter according to item type
        const int item_type(itemType(i));
        switch (item_type) {
        case PointedItem:
          painter.setPen(pen_pointed);
          break;
        case SelectedItem:
          painter.setPen(pen_selected);
          break;
        default:
          ROS_ERROR_STREAM("ImageDrawer::drawText(): unexpected item type ("
                           << item_type << "). Will fallback to the default.");
        case DefaultItem:
          painter.setPen(pen_default);
          break;
        }
        // draw the item text
        QRect rect;
        const QString item(QString::fromStdString(state_.items[i]));
        rect = font_metrics.boundingRect(item);
        rect.moveCenter(relativeItemCenter(i));
        rect.translate(image_center);
        painter.drawText(rect, Qt::AlignCenter, item);
      }
    }

    // draw the title text at the image center
    if (!state_.title.empty()) {
      painter.setPen(makeColor(prop_.title_rgb, prop_.text_alpha));
      QRect rect;
      const QString title(QString::fromStdString(state_.title));
      rect = font_metrics.boundingRect(title);
      rect.moveCenter(image_center);
      painter.drawText(rect, Qt::AlignCenter, title);
    }
  }

  // helper functions

  QSize imageSize() const {
    const int len(2 * (prop_.title_area_radius + prop_.line_width + prop_.item_area_width));
    return QSize(len, len);
  }

  ItemType itemType(const int item_id) const {
    if (item_id < 0 || item_id >= state_.items.size()) {
      return InvalidItem;
    } else if (item_id == state_.pointed_id) {
      return PointedItem;
    } else if (std::find(state_.selected_ids.begin(), state_.selected_ids.end(), item_id) !=
               state_.selected_ids.end()) {
      return SelectedItem;
    } else {
      return DefaultItem;
    }
  }

  // angle for QPainter (0 at 3 o'clock, counterclockwise positive, in 1/16 degrees)
  int itemCenterAngle(const int item_id) const {
    return state_.items.empty() ? 0 : (360 * 16 * item_id / state_.items.size() + 90 * 16);
  }

  // angle for QPainter (in 1/16 degrees)
  int itemSpanAngle() const { return state_.items.empty() ? 0 : (360 * 16 / state_.items.size()); }

  // calc the end position of the line between i-th and (i+1)-th item areas,
  // relative to the menu center
  QPoint relativeItemLineEnd(const int item_id) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const double th(state_.items.empty() ? 0.
                                         : (2. * M_PI * (0.5 + item_id) / state_.items.size()));
    // upward & leftward positive
    const double radius(prop_.title_area_radius + prop_.line_width + prop_.item_area_width);
    const double u(radius * std::cos(th)), v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

  // calc the center position of item text, relative to the menu center
  QPoint relativeItemCenter(const int item_id) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const double th(state_.items.empty() ? 0. : (2. * M_PI * item_id / state_.items.size()));
    // upward & leftward positive
    const double radius(prop_.title_area_radius + prop_.line_width + prop_.item_area_width / 2.);
    const double u(radius * std::cos(th)), v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

  static QColor makeColor(const QRgb &rgb, const int alpha) {
    QColor color;
    color.setRgb(rgb);
    color.setAlpha(alpha);
    return color;
  }

protected:
  radial_menu_msgs::State state_;
  DrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif