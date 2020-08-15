#ifndef RADIAL_MENU_RVIZ_HORIZONTAL_DISPLAY_HPP
#define RADIAL_MENU_RVIZ_HORIZONTAL_DISPLAY_HPP

#include <radial_menu_rviz/display_base.hpp>
#include <radial_menu_rviz/horizontal_image_drawer.hpp>
#include <radial_menu_rviz/horizontal_property_control.hpp>
#include <radial_menu_rviz/properties.hpp>

#include <QObject>

namespace radial_menu_rviz {

class HorizontalDisplay : public DisplayBase< HorizontalDrawingProperty, HorizontalPropertyControl,
                                              HorizontalImageDrawer > {
  Q_OBJECT

private:
  typedef DisplayBase< HorizontalDrawingProperty, HorizontalPropertyControl, HorizontalImageDrawer >
      Base;

public:
  HorizontalDisplay() {}

  virtual ~HorizontalDisplay() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    Base::onInitialize();

    // origin in the position property indicates top-left position
    overlay_->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    // slots on properties changed
    connect(prop_ctl_.get(), &HorizontalPropertyControl::descriptionPropertyChanged, this,
            &HorizontalDisplay::updateDescription);
    connect(prop_ctl_.get(), &HorizontalPropertyControl::subscriptionPropertyChanged, this,
            &HorizontalDisplay::updateSubscription);
    connect(prop_ctl_.get(), &HorizontalPropertyControl::drawingPropertyChanged, this,
            &HorizontalDisplay::updateImage);
    connect(prop_ctl_.get(), &HorizontalPropertyControl::positionPropertyChanged, this,
            &HorizontalDisplay::updatePosition);
  }

protected Q_SLOTS:
  void updateDescription(const DescriptionProperty &prop) { Base::updateDescription(prop); }

  void updateSubscription(const SubscriptionProperty &prop) { Base::updateSubscription(prop); }

  void updateImage(const HorizontalDrawingProperty &prop) { Base::updateImage(prop); }

  void updatePosition(const PositionProperty &prop) { Base::updatePosition(prop); }
};
} // namespace radial_menu_rviz

#endif