#ifndef RADIAL_MENU_RVIZ_PROPERTY_CONTROL_HPP
#define RADIAL_MENU_RVIZ_PROPERTY_CONTROL_HPP

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/properties.hpp>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>

#include <QFontDatabase>
#include <QStringList>

#include <boost/scoped_ptr.hpp>

namespace radial_menu_rviz {

class PropertyControl : public QObject {
  Q_OBJECT

public:
  PropertyControl(rviz::Property *const parent) {
    // subscription control
    state_topic_ctl_.reset(new rviz::RosTopicProperty(
        /* name = */ "State Topic", /* default val = */ "",
        /* msg type = */ ros::message_traits::datatype< radial_menu_msgs::State >(),
        /* desc = */ "Subscribed topic of radial_menu_msgs::State to visualize",
        /* parent = */ parent,
        /* slot = */ SLOT(updateSubscriptionProperty()), /* receiver = */ this));

    // drawing control (background color)
    bg_rgb_ctl_.reset(new rviz::ColorProperty("Background RGB", QColor(0, 0, 0), "Background RGB",
                                              parent, SLOT(updateDrawingProperty()), this));
    bg_alpha_ctl_.reset(new rviz::FloatProperty("Background alpha", 0.8, "Background alpha", parent,
                                                SLOT(updateDrawingProperty()), this));
    bg_alpha_ctl_->setMin(0.);
    bg_alpha_ctl_->setMax(1.);

    // drawing control (font)
    font_ctl_.reset(new rviz::EnumProperty("Font", "DejaVu Sans Mono", "Font", parent,
                                           SLOT(updateDrawingProperty()), this));
    const QStringList font_families_(QFontDatabase().families());
    for (int i = 0; i < font_families_.size(); ++i) {
      font_ctl_->addOption(font_families_[i], i);
    }
    font_size_ctl_.reset(new rviz::IntProperty("Font size", 12, "Font size in points", parent,
                                               SLOT(updateDrawingProperty()), this));

    // drawing control (text color)
    txt_rgb_default_ctl_.reset(new rviz::ColorProperty("Text RGB (default)", QColor(84, 84, 121),
                                                       "Text RGB (default)", parent,
                                                       SLOT(updateDrawingProperty()), this));
    txt_rgb_pointed_ctl_.reset(new rviz::ColorProperty("Text RGB (pointed)", QColor(136, 181, 160),
                                                       "Text RGB (pointed)", parent,
                                                       SLOT(updateDrawingProperty()), this));
    txt_rgb_selected_ctl_.reset(new rviz::ColorProperty("Text RGB (selected)", QColor(25, 255, 240),
                                                        "Text RGB (selected)", parent,
                                                        SLOT(updateDrawingProperty()), this));
    txt_alpha_ctl_.reset(new rviz::FloatProperty("Text alpha", 0.8, "Text alpha", parent,
                                                 SLOT(updateDrawingProperty()), this));
    txt_alpha_ctl_->setMin(0.);
    txt_alpha_ctl_->setMax(1.);

    // drawing control (frame)
    radius_ctl_.reset(new rviz::IntProperty("Radius", 128, "Radius of menu in pixels", parent,
                                            SLOT(updateDrawingProperty()), this));
    radius_ctl_->setMin(0);
    padding_ctl_.reset(new rviz::IntProperty("Padding", 32, "Padding around menu in pixels",
                                             parent, SLOT(updateDrawingProperty()), this));
    padding_ctl_->setMin(0);

    // position control
    left_ctl_.reset(new rviz::IntProperty("Left", 128, "Left position of menu in pixels", parent,
                                          SLOT(updatePositionProperty()), this));
    left_ctl_->setMin(0);
    top_ctl_.reset(new rviz::IntProperty("Top", 128, "Top position of menu in pixels", parent,
                                         SLOT(updatePositionProperty()), this));
    top_ctl_->setMin(0);

    // manually call slots to populate the initial properties
    updateSubscriptionProperty();
    updateDrawingProperty();
    updatePositionProperty();
  }

  virtual ~PropertyControl() {}

  const SubscriptionProperty &subscriptionProperty() const { return sub_prop_; }

  const DrawingProperty &drawingProperty() const { return drawing_prop_; }

  const PositionProperty &positionProperty() const { return pos_prop_; }

Q_SIGNALS:
  void subscriptionPropertyChanged(const SubscriptionProperty &prop);
  void drawingPropertyChanged(const DrawingProperty &prop);
  void positionPropertyChanged(const PositionProperty &prop);

protected Q_SLOTS:
  void updateSubscriptionProperty() {
    sub_prop_.topic = state_topic_ctl_->getTopic();

    Q_EMIT subscriptionPropertyChanged(sub_prop_);
  }

  void updateDrawingProperty() {
    drawing_prop_.bg_color.setRgb(bg_rgb_ctl_->getColor().rgb());
    drawing_prop_.bg_color.setAlphaF(bg_alpha_ctl_->getFloat());

    drawing_prop_.font.setFamily(font_ctl_->getString());
    drawing_prop_.font.setPointSize(font_size_ctl_->getInt());

    drawing_prop_.txt_color_default.setRgb(txt_rgb_default_ctl_->getColor().rgb());
    drawing_prop_.txt_color_default.setAlphaF(txt_alpha_ctl_->getFloat());

    drawing_prop_.txt_color_pointed.setRgb(txt_rgb_pointed_ctl_->getColor().rgb());
    drawing_prop_.txt_color_pointed.setAlphaF(txt_alpha_ctl_->getFloat());

    drawing_prop_.txt_color_selected.setRgb(txt_rgb_selected_ctl_->getColor().rgb());
    drawing_prop_.txt_color_selected.setAlphaF(txt_alpha_ctl_->getFloat());

    drawing_prop_.radius = radius_ctl_->getInt();

    drawing_prop_.padding = padding_ctl_->getInt();

    Q_EMIT drawingPropertyChanged(drawing_prop_);
  }

  void updatePositionProperty() {
    pos_prop_.top_left.setX(left_ctl_->getInt());
    pos_prop_.top_left.setY(top_ctl_->getInt());

    Q_EMIT positionPropertyChanged(pos_prop_);
  }

protected:
  // subscription property & control
  boost::scoped_ptr< rviz::RosTopicProperty > state_topic_ctl_;
  SubscriptionProperty sub_prop_;

  // drawing property & control
  boost::scoped_ptr< rviz::ColorProperty > bg_rgb_ctl_;
  boost::scoped_ptr< rviz::FloatProperty > bg_alpha_ctl_;
  boost::scoped_ptr< rviz::EnumProperty > font_ctl_;
  boost::scoped_ptr< rviz::IntProperty > font_size_ctl_;
  boost::scoped_ptr< rviz::ColorProperty > txt_rgb_default_ctl_, txt_rgb_pointed_ctl_,
      txt_rgb_selected_ctl_;
  boost::scoped_ptr< rviz::FloatProperty > txt_alpha_ctl_;
  boost::scoped_ptr< rviz::IntProperty > radius_ctl_, padding_ctl_;
  DrawingProperty drawing_prop_;

  // position property & control
  boost::scoped_ptr< rviz::IntProperty > left_ctl_, top_ctl_;
  PositionProperty pos_prop_;
};
} // namespace radial_menu_rviz

#endif