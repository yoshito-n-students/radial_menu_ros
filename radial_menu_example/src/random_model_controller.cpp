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
#include <ros/time.h>

namespace rmm = radial_menu_model;

// *******************
// Random number utils
// *******************

std::default_random_engine &randomEngine() {
  static std::random_device seed_gen;
  static std::default_random_engine engine(seed_gen());
  return engine;
}

int randomSid(const int n_sibilings) {
  return std::uniform_int_distribution< int >(0, n_sibilings - 1)(randomEngine());
}

// **************************
// Operations on a menu model
// **************************

bool point(rmm::Model *const model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canPoint(sid)) {
    ROS_INFO_STREAM("Pointing sibiling[" << sid << "] ...");
    model->point(sid);
    return true;
  } else {
    return false;
  }
}

bool unpoint(rmm::Model *const model) {
  const int sid(model->pointedSibilingId());
  if (model->canUnpoint(sid)) {
    ROS_INFO_STREAM("Unpointing sibiling[" << sid << "] ...");
    model->unpoint(sid);
    return true;
  } else {
    return false;
  }
}

bool select(rmm::Model *const model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canSelect(sid)) {
    ROS_INFO_STREAM("Selecting sibiling[" << sid << "] ...");
    model->select(sid, /* allow_multi_selection = */ true);
    return true;
  } else {
    return false;
  }
}

bool deselect(rmm::Model *const model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canDeselect(sid)) {
    ROS_INFO_STREAM("Deselecting sibiling[" << sid << "] ...");
    model->deselect(sid);
    return true;
  } else {
    return false;
  }
}

bool descend(rmm::Model *const model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canDescend(sid)) {
    ROS_INFO_STREAM("Descending from sibiling[" << sid << "] ...");
    model->descend(sid, /* allow_multi_selection = */ true);
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
  ROS_INFO_STREAM("State:\n" << model.state());

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
        ROS_INFO_STREAM("State:\n" << model.state());
        break;
      }
    }
    //
    const radial_menu_msgs::StatePtr state(new radial_menu_msgs::State(model.state()));
    state->header.stamp = ros::Time::now();
    state_pub.publish(state);
    rate.sleep();
  }

  return 0;
}