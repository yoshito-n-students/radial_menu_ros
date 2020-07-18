#ifndef RADIAL_MENU_RVIZ_PROPERTIES_HPP
#define RADIAL_MENU_RVIZ_PROPERTIES_HPP

#include <QFont>
#include <QPoint>
#include <QRgb>
#include <QString>

namespace radial_menu_rviz {

struct SubscriptionProperty {
  QString topic;
};

struct DrawingProperty {
  QFont font;

  int title_area_radius;
  QRgb title_bg_rgb, title_rgb;

  int line_width;

  int item_area_width;
  QRgb item_bg_rgb_default, item_rgb_default;
  QRgb item_bg_rgb_pointed, item_rgb_pointed;
  QRgb item_bg_rgb_selected, item_rgb_selected;

  int bg_alpha, text_alpha;
};

struct PositionProperty {
  QPoint top_left;
};

} // namespace radial_menu_rviz

#endif