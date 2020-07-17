#ifndef RADIAL_MENU_RVIZ_IMAGE_DRAWER_HPP
#define RADIAL_MENU_RVIZ_IMAGE_DRAWER_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <ros/console.h>

#include <QFontMetrics>
#include <QImage>
#include <QMargins>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QString>
#include <QStringList>

namespace radial_menu_rviz {

class ImageDrawer {
public:
  ImageDrawer(const radial_menu_msgs::State &state, const DrawingProperty &prop)
      : state_(state), prop_(prop) {}

  virtual ~ImageDrawer() {}

  void setState(const radial_menu_msgs::State &state) { state_ = state; }

  void setProperty(const DrawingProperty &prop) { prop_ = prop; }

  QImage draw() const {
    // draw nothing on closed state
    switch (state_.state) {
    case radial_menu_msgs::State::STATE_OPENED:
      // just go ahead
      break;
    default:
      ROS_ERROR_STREAM("ImageDrawer::draw(): unexpected menu state ("
                       << state_.state << "). Will fallback to closed state.");
    case radial_menu_msgs::State::STATE_CLOSED:
      return QImage();
    }

    // convert item texts to QString for convenience
    QStringList items;
    for (std::size_t i = 0; i < state_.items.size(); ++i) {
      items.append(QString::fromStdString(state_.items[i]));
    }

    // find the dimension of image and the bounging rectangles of items
    QSize image_size;
    std::vector< QRect > item_rects(items.size());
    {
      // find the dimension of image by uniting bounding rectangles of items
      const QFontMetrics font_metrics(prop_.font);
      const int radius(prop_.title_area_radius + prop_.line_width + prop_.item_area_width / 2);
      QRect image_rect(/* top left = */ QPoint(-radius, -radius),
                       /* bottom right = */ QPoint(radius, radius));
      for (std::size_t i = 0; i < items.size(); ++i) {
        item_rects[i] = font_metrics.boundingRect(items[i]);
        item_rects[i].moveCenter(calcItemCenter(radius, i, items.size()));
        image_rect |= item_rects[i];
      }
      image_rect += QMargins(prop_.item_area_width / 2, prop_.item_area_width / 2,
                             prop_.item_area_width / 2, prop_.item_area_width / 2);
      image_size = image_rect.size();

      // move bounding rectangles of items to image coord
      image_rect.moveTopLeft(QPoint(0, 0));
      for (std::size_t i = 0; i < items.size(); ++i) {
        item_rects[i].translate(image_rect.center());
      }
    }

    // format a new image in the background color
    QImage image(ImageOverlay::formattedImage(image_size, prop_.item_bg_color_default));

    // prepare an image painter and pens
    QPainter painter(&image);
    painter.setFont(prop_.font);
    const QPen pen_default(prop_.item_color_default);
    const QPen pen_pointed(prop_.item_color_pointed);
    const QPen pen_selected(prop_.item_color_selected);

    // draw item texts
    for (std::size_t i = 0; i < items.size(); ++i) {
      // select pen according to item status
      if (std::find(state_.selected_ids.begin(), state_.selected_ids.end(), i) !=
          state_.selected_ids.end()) {
        painter.setPen(pen_selected);
      } else if (state_.pointed_id == i) {
        painter.setPen(pen_pointed);
      } else {
        painter.setPen(pen_default);
      }

      // draw the text
      painter.drawText(item_rects[i], Qt::AlignCenter, items[i]);
    }

    return image;
  }

protected:
  // calc the center position of item text, relative to the image center
  static QPoint calcItemCenter(const int radius, const int item_id, const int item_size) {
    // 0 when upward, counterclockwise positive
    const double th(2. * M_PI * item_id / item_size);
    // upward positive
    const double u(radius * std::cos(th));
    // leftward positive
    const double v(radius * std::sin(th));
    // rightward & downward positive
    return QPoint(-static_cast< int >(v), -static_cast< int >(u));
  }

protected:
  radial_menu_msgs::State state_;
  DrawingProperty prop_;
};
} // namespace radial_menu_rviz

#endif