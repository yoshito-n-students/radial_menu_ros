#ifndef RADIAL_MENU_RVIZ_RADIAL_PROPERTY_CONTROL_HPP
#define RADIAL_MENU_RVIZ_RADIAL_PROPERTY_CONTROL_HPP

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/properties.hpp>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>

#include <QFontDatabase>
#include <QStringList>

#include <boost/scoped_ptr.hpp>

namespace radial_menu_rviz {

class RadialPropertyControl : public QObject {
  Q_OBJECT

public:
  RadialPropertyControl(rviz::Property *const parent) {
    // subscription control
    state_topic_ctl_.reset(new rviz::RosTopicProperty(
        /* name = */ "State topic", /* default val = */ "",
        /* msg type = */ ros::message_traits::datatype< radial_menu_msgs::State >(),
        /* desc = */ "Subscribed topic of radial_menu_msgs::State to visualize",
        /* parent = */ parent,
        /* slot = */ SLOT(updateSubscriptionProperty()), /* receiver = */ this));

    // drawing control (font)
    font_ctl_.reset(new rviz::EnumProperty("Font", "DejaVu Sans Mono", "", parent,
                                           SLOT(updateDrawingProperty()), this));
    const QStringList font_families_(QFontDatabase().families());
    for (int i = 0; i < font_families_.size(); ++i) {
      font_ctl_->addOption(font_families_[i], i);
    }
    font_bold_ctl_.reset(
        new rviz::BoolProperty("Font bold", true, "", parent, SLOT(updateDrawingProperty()), this));
    font_size_ctl_.reset(new rviz::IntProperty("Font size", 12, "In points", parent,
                                               SLOT(updateDrawingProperty()), this));

    // drawing control (title area)
    draw_title_area_ctl_.reset(new rviz::BoolProperty("Title area", true, "", parent,
                                                      SLOT(updateDrawingProperty()), this));
    title_area_radius_ctl_.reset(new rviz::IntProperty(
        "Title area radius", 128, "In pixels", parent, SLOT(updateDrawingProperty()), this));
    title_area_radius_ctl_->setMin(1);
    title_bg_rgb_ctl_.reset(new rviz::ColorProperty("Title bg", QColor(0, 0, 0),
                                                    "RGB of title background", parent,
                                                    SLOT(updateDrawingProperty()), this));
    title_rgb_ctl_.reset(new rviz::ColorProperty("Title", QColor(255, 255, 255),
                                                 "RGB of title text", parent,
                                                 SLOT(updateDrawingProperty()), this));

    // drawing control (line)
    line_width_ctl_.reset(new rviz::IntProperty("Line width", 2,
                                                "Width of line between areas in pixels", parent,
                                                SLOT(updateDrawingProperty()), this));
    line_width_ctl_->setMin(0);

    // drawing control (item area)
    item_area_width_ctl_.reset(new rviz::IntProperty("Item area width", 128, "In pixels", parent,
                                                     SLOT(updateDrawingProperty()), this));
    item_area_width_ctl_->setMin(1);

    item_bg_rgb_default_ctl_.reset(
        new rviz::ColorProperty("Item bg (default)", QColor(255, 255, 255),
                                "RGB of item background when not pointed or selected", parent,
                                SLOT(updateDrawingProperty()), this));
    item_rgb_default_ctl_.reset(new rviz::ColorProperty(
        "Item (default)", QColor(0, 0, 0), "RGB of item text when not pointed or selected", parent,
        SLOT(updateDrawingProperty()), this));
    item_bg_rgb_pointed_ctl_.reset(
        new rviz::ColorProperty("Item bg (pointed)", QColor(128, 128, 128),
                                "RGB to be blended to item background when pointed", parent,
                                SLOT(updateDrawingProperty()), this));
    item_rgb_pointed_ctl_.reset(new rviz::ColorProperty(
        "Item (pointed)", QColor(0, 0, 0), "RGB to be blended to item text when pointed", parent,
        SLOT(updateDrawingProperty()), this));
    item_bg_rgb_selected_ctl_.reset(new rviz::ColorProperty(
        "Item bg (selected)", QColor(0, 0, 0), "RGB of item background when selected", parent,
        SLOT(updateDrawingProperty()), this));
    item_rgb_selected_ctl_.reset(new rviz::ColorProperty("Item (selected)", QColor(255, 255, 255),
                                                         "RGB of item text when selected", parent,
                                                         SLOT(updateDrawingProperty()), this));

    // drawing control (alpha)
    bg_alpha_ctl_.reset(new rviz::IntProperty(
        "Bg alpha", 255, "Alpha of all background colors from 0 (transparent) to 255 (opaque)",
        parent, SLOT(updateDrawingProperty()), this));
    bg_alpha_ctl_->setMin(0);
    bg_alpha_ctl_->setMax(255);
    text_alpha_ctl_.reset(new rviz::IntProperty(
        "Text alpha", 255, "Alpha of all text colors from 0 (transparent) to 255 (opaque)", parent,
        SLOT(updateDrawingProperty()), this));
    text_alpha_ctl_->setMin(0);
    text_alpha_ctl_->setMax(255);

    // position control
    center_x_ctl_.reset(new rviz::IntProperty("Center x", 256,
                                              "X position of menu's center in pixels", parent,
                                              SLOT(updatePositionProperty()), this));
    center_x_ctl_->setMin(0);
    center_y_ctl_.reset(new rviz::IntProperty("Center y", 256,
                                              "Y position of menu's center in pixels", parent,
                                              SLOT(updatePositionProperty()), this));
    center_y_ctl_->setMin(0);

    // manually call slots to populate the initial properties
    updateSubscriptionProperty();
    updateDrawingProperty();
    updatePositionProperty();
  }

  virtual ~RadialPropertyControl() {}

  const SubscriptionProperty &subscriptionProperty() const { return sub_prop_; }

  const RadialDrawingProperty &drawingProperty() const { return drawing_prop_; }

  const PositionProperty &positionProperty() const { return pos_prop_; }

Q_SIGNALS:
  void subscriptionPropertyChanged(const SubscriptionProperty &prop);
  void drawingPropertyChanged(const RadialDrawingProperty &prop);
  void positionPropertyChanged(const PositionProperty &prop);

protected Q_SLOTS:
  void updateSubscriptionProperty() {
    sub_prop_.topic = state_topic_ctl_->getTopic();

    Q_EMIT subscriptionPropertyChanged(sub_prop_);
  }

  void updateDrawingProperty() {
    drawing_prop_.font.setFamily(font_ctl_->getString());
    drawing_prop_.font.setBold(font_bold_ctl_->getBool());
    drawing_prop_.font.setPointSize(font_size_ctl_->getInt());

    drawing_prop_.draw_title_area = draw_title_area_ctl_->getBool();
    drawing_prop_.title_area_radius = title_area_radius_ctl_->getInt();
    drawing_prop_.title_bg_rgb = title_bg_rgb_ctl_->getColor().rgb();
    drawing_prop_.title_rgb = title_rgb_ctl_->getColor().rgb();

    drawing_prop_.line_width = line_width_ctl_->getInt();

    drawing_prop_.item_area_width = item_area_width_ctl_->getInt();
    drawing_prop_.item_bg_rgb_default = item_bg_rgb_default_ctl_->getColor().rgb();
    drawing_prop_.item_rgb_default = item_rgb_default_ctl_->getColor().rgb();
    drawing_prop_.item_bg_rgb_pointed = item_bg_rgb_pointed_ctl_->getColor().rgb();
    drawing_prop_.item_rgb_pointed = item_rgb_pointed_ctl_->getColor().rgb();
    drawing_prop_.item_bg_rgb_selected = item_bg_rgb_selected_ctl_->getColor().rgb();
    drawing_prop_.item_rgb_selected = item_rgb_selected_ctl_->getColor().rgb();

    drawing_prop_.bg_alpha = bg_alpha_ctl_->getInt();
    drawing_prop_.text_alpha = text_alpha_ctl_->getInt();

    Q_EMIT drawingPropertyChanged(drawing_prop_);
  }

  void updatePositionProperty() {
    pos_prop_.origin.setX(center_x_ctl_->getInt());
    pos_prop_.origin.setY(center_y_ctl_->getInt());

    Q_EMIT positionPropertyChanged(pos_prop_);
  }

protected:
  // subscription property & control
  boost::scoped_ptr< rviz::RosTopicProperty > state_topic_ctl_;
  SubscriptionProperty sub_prop_;

  // drawing property & control
  // - font
  boost::scoped_ptr< rviz::EnumProperty > font_ctl_;
  boost::scoped_ptr< rviz::BoolProperty > font_bold_ctl_;
  boost::scoped_ptr< rviz::IntProperty > font_size_ctl_;
  // - title area
  boost::scoped_ptr< rviz::BoolProperty > draw_title_area_ctl_;
  boost::scoped_ptr< rviz::IntProperty > title_area_radius_ctl_;
  boost::scoped_ptr< rviz::ColorProperty > title_bg_rgb_ctl_, title_rgb_ctl_;
  // - line
  boost::scoped_ptr< rviz::IntProperty > line_width_ctl_;
  // - item area
  boost::scoped_ptr< rviz::IntProperty > item_area_width_ctl_;
  boost::scoped_ptr< rviz::ColorProperty > item_bg_rgb_default_ctl_, item_rgb_default_ctl_;
  boost::scoped_ptr< rviz::ColorProperty > item_bg_rgb_pointed_ctl_, item_rgb_pointed_ctl_;
  boost::scoped_ptr< rviz::ColorProperty > item_bg_rgb_selected_ctl_, item_rgb_selected_ctl_;
  // - others
  boost::scoped_ptr< rviz::IntProperty > bg_alpha_ctl_, text_alpha_ctl_;
  RadialDrawingProperty drawing_prop_;

  // position property & control
  boost::scoped_ptr< rviz::IntProperty > center_x_ctl_, center_y_ctl_;
  PositionProperty pos_prop_;
};
} // namespace radial_menu_rviz

#endif