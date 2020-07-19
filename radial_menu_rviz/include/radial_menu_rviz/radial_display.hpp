#ifndef RADIAL_MENU_RVIZ_RADIAL_DISPLAY_HPP
#define RADIAL_MENU_RVIZ_RADIAL_DISPLAY_HPP

#include <radial_menu_rviz/display_base.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <radial_menu_rviz/radial_image_drawer.hpp>
#include <radial_menu_rviz/radial_property_control.hpp>

#include <QObject>

namespace radial_menu_rviz {

class RadialDisplay
    : public DisplayBase< RadialDrawingProperty, RadialPropertyControl, RadialImageDrawer > {
  Q_OBJECT

private:
  typedef DisplayBase< RadialDrawingProperty, RadialPropertyControl, RadialImageDrawer > Base;

public:
  RadialDisplay() {}

  virtual ~RadialDisplay() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    Base::onInitialize();

    // slots on properties changed
    connect(prop_ctl_.get(), SIGNAL(subscriptionPropertyChanged(const SubscriptionProperty &)),
            this, SLOT(updateSubscription(const SubscriptionProperty &)));
    connect(prop_ctl_.get(), SIGNAL(drawingPropertyChanged(const RadialDrawingProperty &)), this,
            SLOT(updateImage(const RadialDrawingProperty &)));
    connect(prop_ctl_.get(), SIGNAL(positionPropertyChanged(const PositionProperty &)), this,
            SLOT(updatePosition(const PositionProperty &)));
  }

protected Q_SLOTS:
  void updateSubscription(const SubscriptionProperty &prop) { Base::updateSubscription(prop); }

  void updateImage(const RadialDrawingProperty &prop) { Base::updateImage(prop); }

  void updatePosition(const PositionProperty &prop) { Base::updatePosition(prop); }
};
} // namespace radial_menu_rviz

#endif