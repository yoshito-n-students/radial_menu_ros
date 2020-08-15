#ifndef RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_RADIAL_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_model/model.hpp>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>
#include <rviz/load_resource.h>

#include <QBrush>
#include <QColor>
#include <QImage>
#include <QPaintDevice>
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
    drawForegrounds(&image);
    return image;
  }

protected:
  // drawing functions

  void drawBackgrounds(QImage *const image) const {
    // prepare alpha channel to separately draw
    QImage alpha_image(image->size(), QImage::Format_Grayscale8);
    alpha_image.fill(QColor(0, 0, 0)); // defaultly transparent

    // painters
    QPainter rgb_painter(image), alpha_painter(&alpha_image);
    rgb_painter.setRenderHint(QPainter::Antialiasing);
    alpha_painter.setRenderHint(QPainter::Antialiasing);

    // draw item areas from outer to inner, and then the title area
    for (radial_menu_model::ItemConstPtr level = model_->currentLevel(); level != model_->root();
         level = level->parentLevel()) {
      drawItemBackgrounds(&rgb_painter, &alpha_painter, level);
    }
    drawTitleBackground(&rgb_painter, &alpha_painter);

    // merge the alpha channel to the given image
    rgb_painter.end();
    alpha_painter.end();
    image->setAlphaChannel(alpha_image);
  }

  void drawItemBackgrounds(QPainter *const rgb_painter, QPainter *const alpha_painter,
                           const radial_menu_model::ItemConstPtr &level) const {
    // common properties
    const QPoint image_center(deviceCenter(*rgb_painter->device()));
    const int n_sibilings(level->numSibilings()), depth(level->depth());

    // draw item areas by pies
    if (n_sibilings > 0) {
      // set tools for alpha painter
      const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
      alpha_painter->setPen(bg_alpha);
      alpha_painter->setBrush(bg_alpha);
      // pie's property
      QRect rect;
      rect.setSize(imageSize(depth));
      rect.moveCenter(image_center);
      const int span_angle(pieSpanAngle(n_sibilings));
      // draw pies
      for (int sid = 0; sid < n_sibilings; ++sid) {
        const radial_menu_model::ItemConstPtr item(level->sibiling(sid));
        if (!item) {
          continue;
        }
        // set tools for rgb painter according to item type
        const bool is_selected(model_->isSelected(item)), is_pointed(model_->isPointed(item));
        if (is_selected && is_pointed) {
          const QRgb rgb(averagedRgb(prop_.item_bg_rgb_selected, prop_.item_bg_rgb_pointed));
          rgb_painter->setPen(QPen(rgb));
          rgb_painter->setBrush(QBrush(rgb));
        } else if (is_selected && !is_pointed) {
          rgb_painter->setPen(QPen(prop_.item_bg_rgb_selected));
          rgb_painter->setBrush(QBrush(prop_.item_bg_rgb_selected));
        } else if (!is_selected && is_pointed) {
          const QRgb rgb(averagedRgb(prop_.item_bg_rgb_default, prop_.item_bg_rgb_pointed));
          rgb_painter->setPen(QPen(rgb));
          rgb_painter->setBrush(QBrush(rgb));
        } else { // !is_selected && !is_pointed
          rgb_painter->setPen(QPen(prop_.item_bg_rgb_default));
          rgb_painter->setBrush(QBrush(prop_.item_bg_rgb_default));
        }
        // draw a pie
        const int center_angle(pieCenterAngle(sid, n_sibilings));
        const int start_angle(center_angle - span_angle / 2);
        rgb_painter->drawPie(rect, start_angle, span_angle);
        alpha_painter->drawPie(rect, start_angle, span_angle);
      }
    }

    // draw transparent lines between sibiling items
    if (n_sibilings > 0 && prop_.line_width > 0) {
      alpha_painter->setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      for (int sid = 0; sid < n_sibilings; ++sid) {
        alpha_painter->drawLine(image_center,
                                image_center + relativeItemLineEnd(sid, n_sibilings, depth));
      }
    }

    // fill inner area with transparent circle
    {
      alpha_painter->setPen(QPen(QColor(0, 0, 0), prop_.line_width));
      alpha_painter->setBrush(QColor(0, 0, 0));
      QRect rect;
      rect.setSize(imageSize(depth - 1) + QSize(prop_.line_width, prop_.line_width));
      rect.moveCenter(image_center);
      alpha_painter->drawEllipse(rect);
    }
  }

  void drawTitleBackground(QPainter *const rgb_painter, QPainter *const alpha_painter) const {
    // draw the title area by a filled circle
    if (prop_.title_area_radius > 0) {
      // circle properties
      const QPoint image_center(deviceCenter(*rgb_painter->device()));
      const QPoint relative_corner(prop_.title_area_radius, prop_.title_area_radius);
      const QRect rect(image_center - relative_corner, image_center + relative_corner);
      if (prop_.draw_title_area) {
        //
        rgb_painter->setPen(QPen(prop_.title_bg_rgb));
        rgb_painter->setBrush(QBrush(prop_.title_bg_rgb));
        rgb_painter->drawEllipse(rect);
        //
        const QColor bg_alpha(prop_.bg_alpha, prop_.bg_alpha, prop_.bg_alpha);
        alpha_painter->setPen(bg_alpha);
        alpha_painter->setBrush(bg_alpha);
        alpha_painter->drawEllipse(rect);
      } else {
        alpha_painter->setPen(QColor(0, 0, 0));
        alpha_painter->setBrush(QColor(0, 0, 0));
        alpha_painter->drawEllipse(rect);
      }
    }
  }

  void drawForegrounds(QImage *const image) const {
    QPainter painter(image);
    painter.setFont(prop_.font);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setRenderHint(QPainter::Antialiasing);

    for (radial_menu_model::ItemConstPtr level = model_->currentLevel(); level != model_->root();
         level = level->parentLevel()) {
      drawItemForegrounds(&painter, level);
    }
    drawTitleForeground(&painter);
  }

  void drawItemForegrounds(QPainter *const painter,
                           const radial_menu_model::ItemConstPtr &level) const {
    const QPoint image_center(deviceCenter(*painter->device()));
    const int n_sibilings(level->numSibilings()), depth(level->depth());

    // draw item texts
    for (int sid = 0; sid < n_sibilings; ++sid) {
      const radial_menu_model::ItemConstPtr item(level->sibiling(sid));
      if (!item) {
        continue;
      }
      // set the item bounding rect
      QRect rect;
      rect.setWidth(prop_.item_area_width);
      rect.setHeight(prop_.item_area_width);
      rect.moveCenter(relativeItemCenter(sid, n_sibilings, depth));
      rect.translate(image_center);
      // draw the item with color according to item type
      const bool is_selected(model_->isSelected(item)), is_pointed(model_->isPointed(item));
      if (is_selected && is_pointed) {
        drawItemForeground(painter, averagedRgb(prop_.item_rgb_selected, prop_.item_rgb_pointed),
                           rect, item);
      } else if (is_selected && !is_pointed) {
        drawItemForeground(painter, prop_.item_rgb_selected, rect, item);
      } else if (!is_selected && is_pointed) {
        drawItemForeground(painter, averagedRgb(prop_.item_rgb_default, prop_.item_rgb_pointed),
                           rect, item);
      } else { // !is_selected && !is_pointed
        drawItemForeground(painter, prop_.item_rgb_default, rect, item);
      }
    }
  }

  void drawTitleForeground(QPainter *const painter) const {
    if (prop_.draw_title_area) {
      QRect rect;
      rect.setWidth(2 * prop_.title_area_radius);
      rect.setHeight(2 * prop_.title_area_radius);
      rect.moveCenter(deviceCenter(*painter->device()));
      drawItemForeground(painter, prop_.title_rgb, rect, model_->root());
    }
  }

  void drawItemForeground(QPainter *const painter, const QRgb &rgb, const QRect &rect,
                          const radial_menu_model::ItemConstPtr &item) const {
    painter->setPen(makeColor(rgb, prop_.fg_alpha));
    switch (item->displayType()) {
    case radial_menu_model::Item::Name:
      painter->drawText(rect, Qt::AlignCenter | Qt::TextWordWrap,
                        QString::fromStdString(item->name()));
      return;
    case radial_menu_model::Item::AltTxt:
      painter->drawText(rect, Qt::AlignCenter | Qt::TextWordWrap,
                        QString::fromStdString(item->altTxt()));
      return;
    case radial_menu_model::Item::Image:
      painter->drawPixmap(
          rect, rviz::loadPixmap(QString::fromStdString(item->imgURL()), /* fill_cache = */ true));
      return;
    default:
      ROS_ERROR_STREAM("RadialImageDrawer::drawItemForeground(): the item '"
                       << item->name() << "' has unexpected type ("
                       << static_cast< int >(item->displayType()) << ")");
      return;
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

  static QPoint deviceCenter(const QPaintDevice &device) {
    return QPoint(device.width() / 2, device.height() / 2);
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