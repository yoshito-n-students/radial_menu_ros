#include <algorithm>
#include <ctime>
#include <vector>

#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

namespace rmm = radial_menu_model;

// **************************
// Operations on a menu model
// **************************

rmm::ItemConstPtr randomSibiling(const rmm::ItemConstPtr &level) {
  std::vector< rmm::ItemConstPtr > sibilings(level->sibilings());
  std::random_shuffle(sibilings.begin(), sibilings.end());
  for (const rmm::ItemConstPtr &s : sibilings) {
    if (s) {
      return s;
    }
  }
  return rmm::ItemConstPtr();
}

bool point(rmm::Model *const model) {
  const rmm::ItemConstPtr s(randomSibiling(model->currentLevel()));
  if (model->canPoint(s)) {
    ROS_INFO_STREAM("Pointing '" << s->name() << "' ...");
    model->point(s);
    return true;
  } else {
    return false;
  }
}

bool unpoint(rmm::Model *const model) {
  const rmm::ItemConstPtr s(randomSibiling(model->currentLevel()));
  if (model->canUnpoint(s)) {
    ROS_INFO_STREAM("Unpointing '" << s->name() << "' ...");
    model->unpoint(s);
    return true;
  } else {
    return false;
  }
}

bool select(rmm::Model *const model) {
  const rmm::ItemConstPtr s(randomSibiling(model->currentLevel()));
  if (model->canSelect(s)) {
    ROS_INFO_STREAM("Selecting '" << s->name() << "' ...");
    model->select(s, /* allow_multi_selection = */ false);
    return true;
  } else {
    return false;
  }
}

bool deselect(rmm::Model *const model) {
  const rmm::ItemConstPtr s(randomSibiling(model->currentLevel()));
  if (model->canDeselect(s)) {
    ROS_INFO_STREAM("Deselecting '" << s->name() << "' ...");
    model->deselect(s);
    return true;
  } else {
    return false;
  }
}

bool descend(rmm::Model *const model) {
  const rmm::ItemConstPtr s(randomSibiling(model->currentLevel()));
  if (model->canDescend(s)) {
    ROS_INFO_STREAM("Descending from '" << s->name() << "' ...");
    model->descend(s, /* allow_multi_selection = */ false);
    return true;
  } else {
    return false;
  }
}

bool ascend(rmm::Model *const model) {
  if (model->canAscend()) {
    ROS_INFO("Ascending ...");
    model->ascend();
    return true;
  } else {
    return false;
  }
}

// *****************
// The main function
// *****************

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "random_model_controller");
  ros::NodeHandle nh;

  //
  ros::Publisher state_pub(nh.advertise< radial_menu_msgs::State >("menu_state", 1, true));

  // load a menu from param
  rmm::Model model;
  if (!model.setDescriptionFromParam("menu_description")) {
    return 1;
  }
  model.setEnabled(true);
  ROS_INFO_STREAM("Model:\n" << model.toString());
  ROS_INFO_STREAM("State:\n" << *model.exportState());

  // perform random operations on the menu model
  typedef bool (*Operation)(rmm::Model *const);
  std::array< Operation, 6 > operations = {point, unpoint, select, deselect, descend, ascend};
  ros::Rate rate(1.);
  while (ros::ok()) {
    // shuffle operation order and try one by one until the first success
    std::random_shuffle(operations.begin(), operations.end());
    for (const Operation op : operations) {
      if ((*op)(&model)) {
        ROS_INFO_STREAM("Model:\n" << model.toString());
        ROS_INFO_STREAM("State:\n" << *model.exportState());
        break;
      }
    }
    //
    state_pub.publish(model.exportState());
    rate.sleep();
  }

  return 0;
}