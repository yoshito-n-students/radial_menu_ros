#ifndef RADIAL_MENU_RVIZ_DISPLAY_HPP
#define RADIAL_MENU_RVIZ_DISPLAY_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <radial_menu_msgs/State.h>
#include <radial_menu_rviz/image_overlay.hpp>
#include <radial_menu_rviz/properties.hpp>
#include <radial_menu_rviz/property_control.hpp>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/subscriber.h>
#include <rviz/display.h>

#include <QFontMetrics>
#include <QImage>
#include <QMargins>
#include <QPainter>
#include <QPen>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QStringList>

#include <boost/scoped_ptr.hpp>

namespace radial_menu_rviz {

class Display : public rviz::Display {
  Q_OBJECT

public:
  Display() {}

  virtual ~Display() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    namespace rmm = radial_menu_msgs;

    // property control on the Display panel
    prop_ctl_.reset(new PropertyControl(this));

    // overlay on the main view
    overlay_.reset(new ImageOverlay());

    // initial state (closed, no items pointed & selected)
    state_ = createClosedState();

    // slots on properties changed
    connect(prop_ctl_.get(), SIGNAL(subscriptionPropertyChanged(const SubscriptionProperty &)),
            this, SLOT(updateSubscription(const SubscriptionProperty &)));
    connect(prop_ctl_.get(), SIGNAL(drawingPropertyChanged(const DrawingProperty &)), this,
            SLOT(redraw(const DrawingProperty &)));
    connect(prop_ctl_.get(), SIGNAL(positionPropertyChanged(const PositionProperty &)), this,
            SLOT(updatePosition(const PositionProperty &)));

    // manually execute the slots to apply the initial properties
    // (except subscription. it will be executed in onEnable().)
    redraw(prop_ctl_->drawingProperty());
    updatePosition(prop_ctl_->positionProperty());
  }

  // called when enabled
  virtual void onEnable() {
    updateSubscription(prop_ctl_->subscriptionProperty());
    overlay_->show();
  }

  // called when disabled
  virtual void onDisable() {
    overlay_->hide();
    state_sub_.shutdown();
  }

  void updateState(const radial_menu_msgs::StateConstPtr &state) {
    state_ = state;
    redraw(prop_ctl_->drawingProperty());
  }

protected Q_SLOTS:
  void updateSubscription(const SubscriptionProperty &prop) {
    // unsubscribe
    state_sub_.shutdown();

    // destroy the last message from the previous session
    state_ = createClosedState();
    redraw(prop_ctl_->drawingProperty());

    // subscribe the new topic
    try {
      state_sub_ =
          ros::NodeHandle().subscribe(prop.topic.toStdString(), 1, &Display::updateState, this);
    } catch (const ros::Exception &error) {
      ROS_ERROR_STREAM(getName().toStdString()
                       << ": error on subscribing topic ('" << prop.topic.toStdString()
                       << "'): " << error.what());
    }
  }

  void redraw(const DrawingProperty &prop) {
    // draw nothing on closed state
    switch (state_->state) {
    case radial_menu_msgs::State::STATE_OPENED:
      // just go ahead
      break;
    default:
      ROS_ERROR_STREAM(getName().toStdString() << ": unexpected menu state (" << state_->state
                                               << "). Will fallback to closed state.");
    case radial_menu_msgs::State::STATE_CLOSED:
      overlay_->setImage(QImage());
      return;
    }

    // convert item texts to QString for convenience
    QStringList items;
    for (std::size_t i = 0; i < state_->items.size(); ++i) {
      items.append(QString::fromStdString(state_->items[i]));
    }

    // find the dimension of image and the bounging rectangles of items
    QSize image_size;
    std::vector< QRect > item_rects(items.size());
    {
      // find the dimension of image by uniting bounding rectangles of items
      const QFontMetrics font_metrics(prop.font);
      QRect image_rect(/* top left = */ QPoint(-prop.radius, -prop.radius),
                       /* bottom right = */ QPoint(prop.radius, prop.radius));
      for (std::size_t i = 0; i < items.size(); ++i) {
        item_rects[i] = font_metrics.boundingRect(items[i]);
        item_rects[i].moveCenter(calcItemCenter(prop.radius, i, items.size()));
        image_rect |= item_rects[i];
      }
      image_rect += QMargins(prop.padding, prop.padding, prop.padding, prop.padding);
      image_size = image_rect.size();

      // move bounding rectangles of items to image coord
      image_rect.moveTopLeft(QPoint(0, 0));
      for (std::size_t i = 0; i < items.size(); ++i) {
        item_rects[i].translate(image_rect.center());
      }
    }

    // format a new image in the background color
    QImage image(ImageOverlay::formattedImage(image_size, prop.bg_color));

    // prepare an image painter and pens
    QPainter painter(&image);
    painter.setFont(prop.font);
    const QPen pen_default(prop.txt_color_default);
    const QPen pen_pointed(prop.txt_color_pointed);
    const QPen pen_selected(prop.txt_color_selected);

    // draw item texts
    for (std::size_t i = 0; i < items.size(); ++i) {
      // select pen according to item status
      if (std::find(state_->selected_ids.begin(), state_->selected_ids.end(), i) !=
          state_->selected_ids.end()) {
        painter.setPen(pen_selected);
      } else if (state_->pointed_id == i) {
        painter.setPen(pen_pointed);
      } else {
        painter.setPen(pen_default);
      }

      // draw the text
      painter.drawText(item_rects[i], Qt::AlignCenter, items[i]);
    }

    // set the new image to overlay
    overlay_->setImage(image);
  }

  void updatePosition(const PositionProperty &prop) { overlay_->setTopLeft(prop.top_left); }

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

  static radial_menu_msgs::StatePtr createClosedState() {
    namespace rmm = radial_menu_msgs;
    rmm::StatePtr state(new rmm::State);
    state->state = rmm::State::STATE_CLOSED;
    state->pointed_id = -1;
    return state;
  }

protected:
  // property control via Rviz
  boost::scoped_ptr< PropertyControl > prop_ctl_;

  // overlay on Rviz
  boost::scoped_ptr< ImageOverlay > overlay_;

  // menu state to be visualized & its subscriber
  radial_menu_msgs::StateConstPtr state_;
  ros::Subscriber state_sub_;
};
} // namespace radial_menu_rviz

#endif