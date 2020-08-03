#include <algorithm>
#include <ctime>
#include <random>

#include <radial_menu_model/model.hpp>
#include <radial_menu_msgs/State.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>

namespace rmm = radial_menu_model;

// *******************
// Random number utils
// *******************

std::default_random_engine &randomEngine() {
  static std::random_device seed_gen;
  static std::default_random_engine engine(seed_gen());
  return engine;
}

rmm::ItemConstPtr randomSibiling(const rmm::ItemConstPtr &level) {
  const int n_sibilings(level->numSibilings());
  const int random_sid(std::uniform_int_distribution< int >(0, n_sibilings - 1)(randomEngine()));
  return level->sibiling(random_sid);
}

// **************************
// Operations on a menu model
// **************************

bool point(rmm::Model *const model) {
  const rmm::ItemConstPtr sibiling(randomSibiling(model->currentLevel()));
  if (model->canPoint(sibiling)) {
    ROS_INFO_STREAM("Pointing '" << sibiling->name() << "' ...");
    model->point(sibiling);
    return true;
  } else {
    return false;
  }
}

bool unpoint(rmm::Model *const model) {
  const rmm::ItemConstPtr sibiling(randomSibiling(model->currentLevel()));
  if (model->canUnpoint(sibiling)) {
    ROS_INFO_STREAM("Unpointing '" << sibiling->name() << "' ...");
    model->unpoint(sibiling);
    return true;
  } else {
    return false;
  }
}

bool select(rmm::Model *const model) {
  const rmm::ItemConstPtr sibiling(randomSibiling(model->currentLevel()));
  if (model->canSelect(sibiling)) {
    ROS_INFO_STREAM("Selecting '" << sibiling->name() << "' ...");
    model->select(sibiling, /* allow_multi_selection = */ true);
    return true;
  } else {
    return false;
  }
}

bool deselect(rmm::Model *const model) {
  const rmm::ItemConstPtr sibiling(randomSibiling(model->currentLevel()));
  if (model->canDeselect(sibiling)) {
    ROS_INFO_STREAM("Deselecting '" << sibiling->name() << "' ...");
    model->deselect(sibiling);
    return true;
  } else {
    return false;
  }
}

bool descend(rmm::Model *const model) {
  const rmm::ItemConstPtr sibiling(randomSibiling(model->currentLevel()));
  if (model->canDescend(sibiling)) {
    ROS_INFO_STREAM("Descending from '" << sibiling << "' ...");
    model->descend(sibiling, /* allow_multi_selection = */ true);
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
    std::shuffle(operations.begin(), operations.end(), randomEngine());
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