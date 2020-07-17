#ifndef RADIAL_MENU_RVIZ_PROPERTIES_HPP
#define RADIAL_MENU_RVIZ_PROPERTIES_HPP

#include <QColor>
#include <QFont>
#include <QPoint>
#include <QString>

namespace radial_menu_rviz {

struct SubscriptionProperty {
  QString topic;
};

struct DrawingProperty {
  QFont font;

  int title_area_radius;
  QColor title_bg_color;
  QColor title_color;

  int line_width;

  int item_area_width;
  QColor item_bg_color_default;
  QColor item_color_default;
  QColor item_bg_color_pointed;
  QColor item_color_pointed;
  QColor item_bg_color_selected;
  QColor item_color_selected;
};

struct PositionProperty {
  QPoint top_left;
};

} // namespace radial_menu_rviz

#endif