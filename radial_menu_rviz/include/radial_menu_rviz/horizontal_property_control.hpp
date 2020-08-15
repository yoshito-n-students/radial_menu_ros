#ifndef RADIAL_MENU_RVIZ_HORIZONTAL_PROPERTY_CONTROL_HPP
#define RADIAL_MENU_RVIZ_HORIZONTAL_PROPERTY_CONTROL_HPP

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/properties.hpp>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>

#include <QFontDatabase>
#include <QStringList>

namespace radial_menu_rviz {

class HorizontalPropertyControl : public QObject {
  Q_OBJECT

public:
  HorizontalPropertyControl(rviz::Property *const parent) {
    // description control
    desc_param_ctl_.reset(new rviz::StringProperty(
        /* name = */ "Menu description", /* default val = */ "",
        /* desc = */ "ROS parameter describing the menu tree model", /* parent = */ parent,
        /* slot = */ SLOT(updateDescriptionProperty()), /* receiver = */ this));

    // subscription control
    state_topic_ctl_.reset(new rviz::RosTopicProperty(
        "State topic", "",
        /* msg type = */ ros::message_traits::datatype< radial_menu_msgs::State >(),
        "Subscribed topic of radial_menu_msgs::State to visualize", parent,
        SLOT(updateSubscriptionProperty()), this));

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

    // drawing control (title)
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
    item_bg_rgb_pointed_ctl_.reset(new rviz::ColorProperty(
        "Item bg (pointed)", QColor(128, 128, 128), "RGB of item background when pointed", parent,
        SLOT(updateDrawingProperty()), this));
    item_rgb_pointed_ctl_.reset(new rviz::ColorProperty("Item (pointed)", QColor(0, 0, 0),
                                                        "RGB of item text when pointed", parent,
                                                        SLOT(updateDrawingProperty()), this));
    item_bg_rgb_selected_ctl_.reset(new rviz::ColorProperty(
        "Item bg (selected)", QColor(0, 0, 0), "RGB of item background when selected", parent,
        SLOT(updateDrawingProperty()), this));
    item_rgb_selected_ctl_.reset(new rviz::ColorProperty("Item (selected)", QColor(255, 255, 255),
                                                         "RGB of item text when selected", parent,
                                                         SLOT(updateDrawingProperty()), this));

    // drawing control (common for title and items)
    bg_alpha_ctl_.reset(new rviz::IntProperty(
        "Bg alpha", 255, "Alpha of all background colors from 0 (transparent) to 255 (opaque)",
        parent, SLOT(updateDrawingProperty()), this));
    bg_alpha_ctl_->setMin(0);
    bg_alpha_ctl_->setMax(255);
    fg_alpha_ctl_.reset(new rviz::IntProperty(
        "Fg alpha", 255, "Alpha of all foreground colors from 0 (transparent) to 255 (opaque)",
        parent, SLOT(updateDrawingProperty()), this));
    fg_alpha_ctl_->setMin(0);
    fg_alpha_ctl_->setMax(255);
    bg_padding_ctl_.reset(new rviz::IntProperty("Bg padding", 16,
                                                "Padding of all background area in pixels", parent,
                                                SLOT(updateDrawingProperty()), this));
    bg_padding_ctl_->setMin(0);
    fg_height_ctl_.reset(new rviz::IntProperty("Fg height", 32,
                                               "Height of all foreground area in pixels", parent,
                                               SLOT(updateDrawingProperty()), this));
    fg_height_ctl_->setMin(0);

    // position control
    left_ctl_.reset(new rviz::IntProperty("Left", 128, "Position of menu's left edge in pixels",
                                          parent, SLOT(updatePositionProperty()), this));
    left_ctl_->setMin(0);
    top_ctl_.reset(new rviz::IntProperty("Top", 128, "Position of menu's top edge in pixels",
                                         parent, SLOT(updatePositionProperty()), this));
    top_ctl_->setMin(0);

    // manually call slots to populate the initial properties
    updateSubscriptionProperty();
    updateDrawingProperty();
    updatePositionProperty();
  }

  virtual ~HorizontalPropertyControl() {}

  const DescriptionProperty &descriptionProperty() const { return desc_prop_; }

  const SubscriptionProperty &subscriptionProperty() const { return sub_prop_; }

  const HorizontalDrawingProperty &drawingProperty() const { return drawing_prop_; }

  const PositionProperty &positionProperty() const { return pos_prop_; }

Q_SIGNALS:
  void descriptionPropertyChanged(const DescriptionProperty &prop);
  void subscriptionPropertyChanged(const SubscriptionProperty &prop);
  void drawingPropertyChanged(const HorizontalDrawingProperty &prop);
  void positionPropertyChanged(const PositionProperty &prop);

protected Q_SLOTS:
  void updateDescriptionProperty() {
    desc_prop_.param_name = desc_param_ctl_->getString();

    Q_EMIT descriptionPropertyChanged(desc_prop_);
  }

  void updateSubscriptionProperty() {
    sub_prop_.topic = state_topic_ctl_->getTopic();

    Q_EMIT subscriptionPropertyChanged(sub_prop_);
  }

  void updateDrawingProperty() {
    drawing_prop_.font.setFamily(font_ctl_->getString());
    drawing_prop_.font.setBold(font_bold_ctl_->getBool());
    drawing_prop_.font.setPointSize(font_size_ctl_->getInt());

    drawing_prop_.title_bg_rgb = title_bg_rgb_ctl_->getColor().rgb();
    drawing_prop_.title_rgb = title_rgb_ctl_->getColor().rgb();

    drawing_prop_.line_width = line_width_ctl_->getInt();

    drawing_prop_.item_bg_rgb_pointed = item_bg_rgb_pointed_ctl_->getColor().rgb();
    drawing_prop_.item_rgb_pointed = item_rgb_pointed_ctl_->getColor().rgb();
    drawing_prop_.item_bg_rgb_selected = item_bg_rgb_selected_ctl_->getColor().rgb();
    drawing_prop_.item_rgb_selected = item_rgb_selected_ctl_->getColor().rgb();

    drawing_prop_.bg_alpha = bg_alpha_ctl_->getInt();
    drawing_prop_.fg_alpha = fg_alpha_ctl_->getInt();
    drawing_prop_.bg_padding = bg_padding_ctl_->getInt();
    drawing_prop_.fg_height = fg_height_ctl_->getInt();

    Q_EMIT drawingPropertyChanged(drawing_prop_);
  }

  void updatePositionProperty() {
    pos_prop_.origin.setX(left_ctl_->getInt());
    pos_prop_.origin.setY(top_ctl_->getInt());

    Q_EMIT positionPropertyChanged(pos_prop_);
  }

protected:
  // description property & control
  std::unique_ptr< rviz::StringProperty > desc_param_ctl_;
  DescriptionProperty desc_prop_;

  // subscription property & control
  std::unique_ptr< rviz::RosTopicProperty > state_topic_ctl_;
  SubscriptionProperty sub_prop_;

  // drawing property & control
  std::unique_ptr< rviz::EnumProperty > font_ctl_;
  std::unique_ptr< rviz::BoolProperty > font_bold_ctl_;
  std::unique_ptr< rviz::IntProperty > font_size_ctl_;
  std::unique_ptr< rviz::ColorProperty > title_bg_rgb_ctl_, title_rgb_ctl_;
  std::unique_ptr< rviz::IntProperty > line_width_ctl_;
  std::unique_ptr< rviz::ColorProperty > item_bg_rgb_pointed_ctl_, item_rgb_pointed_ctl_;
  std::unique_ptr< rviz::ColorProperty > item_bg_rgb_selected_ctl_, item_rgb_selected_ctl_;
  std::unique_ptr< rviz::IntProperty > bg_alpha_ctl_, fg_alpha_ctl_;
  std::unique_ptr< rviz::IntProperty > bg_padding_ctl_, fg_height_ctl_;
  HorizontalDrawingProperty drawing_prop_;

  // position property & control
  std::unique_ptr< rviz::IntProperty > left_ctl_, top_ctl_;
  PositionProperty pos_prop_;
};
} // namespace radial_menu_rviz

#endif