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
  RadialImageDrawer(const radial_menu_msgs::State &state, const RadialDrawingProperty &prop) {
    setState(state);
    setProperty(prop);
  }

  virtual ~RadialImageDrawer() {}

  void setState(const radial_menu_msgs::State &state) { state_ = state; }

  void setProperty(const RadialDrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    // if state is invalid, draw nothing
    const int max_depth(static_cast< int >(state_.widths.size()) - 1);
    const int max_item_id(static_cast< int >(state_.items.size()) - 1);
    if (max_depth < 0 || lastItemId(max_depth) != max_item_id) {
      return QImage();
    }

    // if the menu is closed, draw nothing
    if (!state_.is_enabled) {
      return QImage();
    }

    // draw menu elements
    QImage image(ImageOverlay::formattedImage(imageSize(max_depth), Qt::transparent));
    drawBackgrounds(&image);
    drawTexts(&image);
    return image;
  }

protected:
  // drawing functions

  void drawBackgrounds(QImage *const image) const {
    // prepare alpha channel to separately draw
    QImage alpha_image(image->size(), QImage::Format_Grayscale8);
    alpha_image.fill(QColor(0, 0, 0)); // defaultly transparent

    // draw item areas from outer to inner, and then the title area
    for (int depth = state_.widths.size() - 1; depth >= 0; --depth) {
      drawItemBackgrounds(image, &alpha_image, depth);
    }
    drawTitleBackground(image, &alpha_image);

    // merge the alpha channel to the given image
    image->setAlphaChannel(alpha_image);
  }

  void drawItemBackgrounds(QImage *const image, QImage *const alpha_image, const int depth) const {
    // painters
    QPainter rgb_painter(image), alpha_painter(alpha_image);
    rgb_painter.setRenderHint(QPainter::Antialiasing);
    alpha_painter.setRenderHint(QPainter::Antialiasing);

    // common properties
    const QPoint image_center(image->rect().center());
    const int first_item_id(firstItemId(depth)), last_item_id(lastItemId(depth));

    // draw item areas by pies
    if (first_item_id <= last_item_id) {
      // set tools for alpha painter
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter.setPen(bg_alpha);
      alpha_painter.setBrush(bg_alpha);
      // pie's property
      QRect rect;
      rect.setSize(imageSize(depth));
      rect.moveCenter(image_center);
      const int span_angle(pieSpanAngle(depth));
      // draw pies
      for (int item_id = first_item_id; item_id <= last_item_id; ++item_id) {
        // set tools for rgb painter according to item type
        const bool is_selected(std::find(state_.selected_ids.begin(), state_.selected_ids.end(),
                                         item_id) != state_.selected_ids.end());
        const bool is_pointed(item_id == state_.pointed_id);
        if (is_selected && is_pointed) {
          const QRgb rgb(averagedRgb(prop_.item_bg_rgb_selected, prop_.item_bg_rgb_pointed));
          rgb_painter.setPen(QPen(rgb));
          rgb_painter.setBrush(QBrush(rgb));
        } else if (is_selected && !is_pointed) {
          rgb_painter.setPen(QPen(prop_.item_bg_rgb_selected));
          rgb_painter.setBrush(QBrush(prop_.item_bg_rgb_selected));
        } else if (!is_selected && is_pointed) {
          const QRgb rgb(averagedRgb(prop_.item_bg_rgb_default, prop_.item_bg_rgb_pointed));
          rgb_painter.setPen(QPen(rgb));
          rgb_painter.setBrush(QBrush(rgb));
        } else { // !is_selected && !is_pointed
          rgb_painter.setPen(QPen(prop_.item_bg_rgb_default));
          rgb_painter.setBrush(QBrush(prop_.item_bg_rgb_default));
        }
        // draw a pie
        const int center_angle(pieCenterAngle(item_id - first_item_id, depth));
        const int start_angle(center_angle - span_angle / 2);
        rgb_painter.drawPie(rect, start_angle, span_angle);
        alpha_painter.drawPie(rect, start_angle, span_angle);
      }
    }

    // draw transparent lines between item areas
    if (first_item_id <= last_item_id && prop_.line_width > 0) {
      alpha_painter.setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      for (int item_id = first_item_id; item_id <= last_item_id; ++item_id) {
        alpha_painter.drawLine(image_center,
                               image_center + relativeItemLineEnd(item_id - first_item_id, depth));
      }
    }

    // fill inner area with transparent circle
    {
      alpha_painter.setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      alpha_painter.setBrush(QColor(0, 0, 0));
      QRect rect;
      rect.setSize(imageSize(depth - 1) + QSize(prop_.line_width, prop_.line_width));
      rect.moveCenter(image_center);
      alpha_painter.drawEllipse(rect);
    }
  }

  void drawTitleBackground(QImage *const image, QImage *const alpha_image) const {
    // draw the title area by a filled circle
    if (prop_.title_area_radius > 0) {
      // painters
      QPainter rgb_painter(image), alpha_painter(alpha_image);
      rgb_painter.setRenderHint(QPainter::Antialiasing);
      alpha_painter.setRenderHint(QPainter::Antialiasing);
      // circle properties
      const QPoint image_center(image->rect().center());
      const QPoint relative_corner(prop_.title_area_radius, prop_.title_area_radius);
      const QRect rect(image_center - relative_corner, image_center + relative_corner);
      if (prop_.draw_title_area) {
        //
        rgb_painter.setPen(QPen(prop_.title_bg_rgb));
        rgb_painter.setBrush(QBrush(prop_.title_bg_rgb));
        rgb_painter.drawEllipse(rect);
        //
        const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
        alpha_painter.setPen(bg_alpha);
        alpha_painter.setBrush(bg_alpha);
        alpha_painter.drawEllipse(rect);
      } else {
        alpha_painter.setPen(QColor(0, 0, 0));
        alpha_painter.setBrush(QColor(0, 0, 0));
        alpha_painter.drawEllipse(rect);
      }
    }
  }

  void drawTexts(QImage *const image) const {
    for (int depth = state_.widths.size() - 1; depth >= 0; --depth) {
      drawItemTexts(image, depth);
    }
    drawTitleText(image);
  }

  void drawItemTexts(QImage *const image, const int depth) const {
    QPainter painter(image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);

    const QFontMetrics font_metrics(prop_.font);
    const QPoint image_center(image->rect().center());
    const int first_item_id(firstItemId(depth)), last_item_id(lastItemId(depth));

    // draw item texts
    for (int item_id = first_item_id; item_id <= last_item_id; ++item_id) {
      // set tools for the painter according to item type
      const bool is_selected(std::find(state_.selected_ids.begin(), state_.selected_ids.end(),
                                       item_id) != state_.selected_ids.end());
      const bool is_pointed(item_id == state_.pointed_id);
      if (is_selected && is_pointed) {
        const QRgb rgb(averagedRgb(prop_.item_rgb_selected, prop_.item_rgb_pointed));
        painter.setPen(makeColor(rgb, prop_.text_alpha));
      } else if (is_selected && !is_pointed) {
        painter.setPen(makeColor(prop_.item_rgb_selected, prop_.text_alpha));
      } else if (!is_selected && is_pointed) {
        const QRgb rgb(averagedRgb(prop_.item_rgb_default, prop_.item_rgb_pointed));
        painter.setPen(makeColor(rgb, prop_.text_alpha));
      } else { // !is_selected && !is_pointed
        painter.setPen(makeColor(prop_.item_rgb_default, prop_.text_alpha));
      }
      // draw the item text
      QRect rect;
      const QString item(QString::fromStdString(state_.items[item_id]));
      rect = font_metrics.boundingRect(item);
      rect.moveCenter(relativeItemCenter(item_id - first_item_id, depth));
      rect.translate(image_center);
      painter.drawText(rect, Qt::AlignCenter, item);
    }
  }

  void drawTitleText(QImage *const image) const {
    if (prop_.draw_title_area && !state_.title.empty()) {
      // painter
      QPainter painter(image);
      painter.setFont(prop_.font);
      painter.setRenderHint(QPainter::TextAntialiasing);
      painter.setPen(makeColor(prop_.title_rgb, prop_.text_alpha));
      // draw the title text at the image center
      QRect rect;
      const QString title(QString::fromStdString(state_.title));
      rect = QFontMetrics(prop_.font).boundingRect(title);
      rect.moveCenter(image->rect().center());
      painter.drawText(rect, Qt::AlignCenter, title);
    }
  }

  // helper functions

  // if the depth is -1, returns the size of title area
  QSize imageSize(const int depth) const {
    const int len(
        2 * (prop_.title_area_radius + (prop_.line_width + prop_.item_area_width) * (depth + 1)));
    return QSize(len, len);
  }

  int menuWidth(const int depth) const {
    return (depth >= 0 && depth < state_.widths.size()) ? state_.widths[depth] : 0;
  }

  int firstItemId(const int depth) const {
    int item_id(0);
    for (int d = 0; d < depth; ++d) {
      item_id += menuWidth(d);
    }
    return item_id;
  }

  int lastItemId(const int depth) const { return firstItemId(depth + 1) - 1; }

  // angle for QPainter (0 at 3 o'clock, counterclockwise positive, in 1/16 degrees)
  int pieCenterAngle(const int id, const int depth) const {
    const int width(menuWidth(depth));
    return (width == 0) ? 0 : (360 * 16 * id / width + 90 * 16);
  }

  // angle for QPainter (in 1/16 degrees)
  int pieSpanAngle(const int depth) const {
    const int width(menuWidth(depth));
    return (width == 0) ? 0 : (360 * 16 / width);
  }

  // calc the end position of the line between i-th and (i+1)-th item areas,
  // relative to the menu center
  QPoint relativeItemLineEnd(const int id, const int depth) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const int width(menuWidth(depth));
    const double th((width == 0) ? 0. : (2. * M_PI * (0.5 + id) / width));
    // upward & leftward positive
    const double radius(prop_.title_area_radius +
                        (prop_.line_width + prop_.item_area_width) * (depth + 1));
    const double u(radius * std::cos(th)), v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

  // calc the center position of item text, relative to the menu center
  QPoint relativeItemCenter(const int id, const int depth) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const int width(menuWidth(depth));
    const double th((width == 0) ? 0. : (2. * M_PI * id / width));
    // upward & leftward positive
    const double radius(prop_.title_area_radius + prop_.line_width * (depth + 1) +
                        prop_.item_area_width * (depth + 0.5));
    const double u(radius * std::cos(th)), v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

  static QRgb averagedRgb(const QRgb &rgb1, const QRgb &rgb2) {
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