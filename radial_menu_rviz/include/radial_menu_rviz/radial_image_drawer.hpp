#ifndef RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_model/model.hpp>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/item_drawer.hpp>
#include <radial_menu_rviz/properties.hpp>

#include <QBrush>
#include <QColor>
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
  RadialImageDrawer(const radial_menu_model::ModelConstPtr &model,
                    const RadialDrawingProperty &prop) {
    setModel(model);
    setProperty(prop);
  }

  virtual ~RadialImageDrawer() {}

  void setModel(const radial_menu_model::ModelConstPtr &model) { model_ = model; }

  void setProperty(const RadialDrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    // if the menu is disabled, draw nothing
    if (!model_->isEnabled()) {
      return QImage();
    }

    // draw menu elements
    QImage image(
        ImageOverlay::formattedImage(imageSize(model_->currentLevel()->depth()), Qt::transparent));
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
    for (radial_menu_model::ItemConstPtr level = model_->currentLevel(); level != model_->root();
         level = level->parentLevel()) {
      drawItemBackgrounds(image, &alpha_image, level);
    }
    drawTitleBackground(image, &alpha_image);

    // merge the alpha channel to the given image
    image->setAlphaChannel(alpha_image);
  }

  void drawItemBackgrounds(QImage *const image, QImage *const alpha_image,
                           const radial_menu_model::ItemConstPtr &level) const {
    // painters
    QPainter rgb_painter(image), alpha_painter(alpha_image);
    rgb_painter.setRenderHint(QPainter::Antialiasing);
    alpha_painter.setRenderHint(QPainter::Antialiasing);

    // common properties
    const QPoint image_center(image->rect().center());
    const int n_sibilings(level->numSibilings()), depth(level->depth());

    // draw item areas by pies
    if (n_sibilings > 0) {
      // set tools for alpha painter
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter.setPen(bg_alpha);
      alpha_painter.setBrush(bg_alpha);
      // pie's property
      QRect rect;
      rect.setSize(imageSize(depth));
      rect.moveCenter(image_center);
      const int span_angle(pieSpanAngle(n_sibilings));
      // draw pies
      for (int sid = 0; sid < n_sibilings; ++sid) {
        // set tools for rgb painter according to item type
        const radial_menu_model::ItemConstPtr item(level->sibiling(sid));
        const bool is_selected(model_->isSelected(item)), is_pointed(model_->isPointed(item));
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
        const int center_angle(pieCenterAngle(sid, n_sibilings));
        const int start_angle(center_angle - span_angle / 2);
        rgb_painter.drawPie(rect, start_angle, span_angle);
        alpha_painter.drawPie(rect, start_angle, span_angle);
      }
    }

    // draw transparent lines between item areas
    if (n_sibilings > 0 && prop_.line_width > 0) {
      alpha_painter.setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      for (int sid = 0; sid < n_sibilings; ++sid) {
        alpha_painter.drawLine(image_center,
                               image_center + relativeItemLineEnd(sid, n_sibilings, depth));
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
    for (radial_menu_model::ItemConstPtr level = model_->currentLevel(); level != model_->root();
         level = level->parentLevel()) {
      drawItemTexts(image, level);
    }
    drawTitleText(image);
  }

  void drawItemTexts(QImage *const image, const radial_menu_model::ItemConstPtr &level) const {
    QPainter painter(image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);

    const QPoint image_center(image->rect().center());
    const int n_sibilings(level->numSibilings()), depth(level->depth());

    // draw item texts
    for (int sid = 0; sid < n_sibilings; ++sid) {
      const radial_menu_model::ItemConstPtr item(level->sibiling(sid));
      // set tools for the painter according to item type
      const bool is_selected(model_->isSelected(item)), is_pointed(model_->isPointed(item));
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
      // set the item bounding rect
      QRect rect;
      rect.setWidth(prop_.item_area_width);
      rect.setHeight(prop_.item_area_width);
      rect.moveCenter(relativeItemCenter(sid, n_sibilings, depth));
      rect.translate(image_center);
      // draw the item
      drawItem(&painter, rect, item);
    }
  }

  void drawTitleText(QImage *const image) const {
    if (prop_.draw_title_area) {
      // painter
      QPainter painter(image);
      painter.setFont(prop_.font);
      painter.setRenderHint(QPainter::TextAntialiasing);
      painter.setPen(makeColor(prop_.title_rgb, prop_.text_alpha));
      // draw the title item at the image center
      QRect rect;
      rect.setWidth(2 * prop_.title_area_radius);
      rect.setHeight(2 * prop_.title_area_radius);
      rect.moveCenter(image->rect().center());
      drawItem(&painter, rect, model_->root());
    }
  }

  // helper functions

  // if the depth is 0, returns the size of title area
  QSize imageSize(const int depth) const {
    const int len(2 *
                  (prop_.title_area_radius + (prop_.line_width + prop_.item_area_width) * depth));
    return QSize(len, len);
  }

  // angle for QPainter (0 at 3 o'clock, counterclockwise positive, in 1/16 degrees)
  int pieCenterAngle(const int sid, const int n_sibilings) const {
    return (n_sibilings == 0) ? 0 : (360 * 16 * sid / n_sibilings + 90 * 16);
  }

  // angle for QPainter (in 1/16 degrees)
  int pieSpanAngle(const int n_sibilings) const {
    return (n_sibilings == 0) ? 0 : (360 * 16 / n_sibilings);
  }

  // calc the end position of the line between i-th and (i+1)-th item areas,
  // relative to the menu center
  QPoint relativeItemLineEnd(const int sid, const int n_sibilings, const int depth) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const double th((n_sibilings == 0) ? 0. : (2. * M_PI * (0.5 + sid) / n_sibilings));
    // upward & leftward positive
    const double radius(prop_.title_area_radius +
                        (prop_.line_width + prop_.item_area_width) * depth);
    const double u(radius * std::cos(th)), v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

  // calc the center position of item text, relative to the menu center
  QPoint relativeItemCenter(const int sid, const int n_sibilings, const int depth) const {
    // 0 at twelve o'clock position, counterclockwise positive
    const double th((n_sibilings == 0) ? 0. : (2. * M_PI * sid / n_sibilings));
    // upward & leftward positive
    const double radius(prop_.title_area_radius + prop_.line_width * depth +
                        prop_.item_area_width * (depth - 0.5));
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
  radial_menu_model::ModelConstPtr model_;
  RadialDrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif