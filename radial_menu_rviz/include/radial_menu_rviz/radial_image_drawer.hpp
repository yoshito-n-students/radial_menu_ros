#ifndef RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>

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

class RadialImageDrawer {
public:
  RadialImageDrawer(const radial_menu_msgs::State &state, const RadialDrawingProperty &prop)
      : state_(state), prop_(prop) {}

  virtual ~RadialImageDrawer() {}

  void setState(const radial_menu_msgs::State &state) { state_ = state; }

  void setProperty(const RadialDrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    if (state_.is_opened) {
      QImage image(ImageOverlay::formattedImage(imageSize(), Qt::transparent));
      drawBackground(&image);
      drawTexts(&image);
      return image;
    } else {
      return QImage();
    }
  }

protected:
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
      // set tools for alpha painter
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter.setPen(bg_alpha);
      alpha_painter.setBrush(bg_alpha);
      // draw each pie
      const int span_angle(itemSpanAngle());
      for (int i = 0; i < state_.items.size(); ++i) {
        // set tools for rgb painter according to item type
        const bool is_selected(std::find(state_.selected_ids.begin(), state_.selected_ids.end(),
                                         i) != state_.selected_ids.end());
        const bool is_pointed(i == state_.pointed_id);
        if (is_selected && is_pointed) {
          const QRgb rgb(blendedRgb(prop_.item_bg_rgb_selected, prop_.item_bg_rgb_pointed));
          rgb_painter.setPen(QPen(rgb));
          rgb_painter.setBrush(QBrush(rgb));
        } else if (is_selected && !is_pointed) {
          rgb_painter.setPen(QPen(prop_.item_bg_rgb_selected));
          rgb_painter.setBrush(QBrush(prop_.item_bg_rgb_selected));
        } else if (!is_selected && is_pointed) {
          const QRgb rgb(blendedRgb(prop_.item_bg_rgb_default, prop_.item_bg_rgb_pointed));
          rgb_painter.setPen(QPen(rgb));
          rgb_painter.setBrush(QBrush(rgb));
        } else { // !is_selected && !is_pointed
          rgb_painter.setPen(QPen(prop_.item_bg_rgb_default));
          rgb_painter.setBrush(QBrush(prop_.item_bg_rgb_default));
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
      for (int i = 0; i < state_.items.size(); ++i) {
        // set tools for the painter according to item type
        const bool is_selected(std::find(state_.selected_ids.begin(), state_.selected_ids.end(),
                                         i) != state_.selected_ids.end());
        const bool is_pointed(i == state_.pointed_id);
        if (is_selected && is_pointed) {
          const QRgb rgb(blendedRgb(prop_.item_rgb_selected, prop_.item_rgb_pointed));
          painter.setPen(makeColor(rgb, prop_.text_alpha));
        } else if (is_selected && !is_pointed) {
          painter.setPen(makeColor(prop_.item_rgb_selected, prop_.text_alpha));
        } else if (!is_selected && is_pointed) {
          const QRgb rgb(blendedRgb(prop_.item_rgb_default, prop_.item_rgb_pointed));
          painter.setPen(makeColor(rgb, prop_.text_alpha));
        } else { // !is_selected && !is_pointed
          painter.setPen(makeColor(prop_.item_rgb_default, prop_.text_alpha));
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

  static QRgb blendedRgb(const QRgb &rgb1, const QRgb &rgb2) {
    const QColor color1(rgb1), color2(rgb2);
    return QColor((color1.red() + color2.red()) / 2, (color1.green() + color2.green()) / 2,
                  (color1.blue() + color2.blue()) / 2)
        .rgb();
  }

  static QColor makeColor(const QRgb &rgb, const int alpha) {
    QColor color;
    color.setRgb(rgb);
    color.setAlpha(alpha);
    return color;
  }

protected:
  radial_menu_msgs::State state_;
  RadialDrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif