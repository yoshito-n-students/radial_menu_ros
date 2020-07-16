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
  QColor bg_color;
  QFont font;
  QColor txt_color_default;
  QColor txt_color_pointed;
  QColor txt_color_selected;
  int radius;
  int padding;
};

struct PositionProperty {
  QPoint top_left;
};

} // namespace radial_menu_rviz

#endif