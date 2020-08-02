#include <algorithm>
#include <ctime>
#include <random>

#include <radial_menu_model/model.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
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

bool point(const rmm::ModelPtr &model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canPoint(sid)) {
    ROS_INFO_STREAM("Pointing sibiling[" << sid << "] ...");
    model->point(sid);
    return true;
  } else {
    return false;
  }
}

bool unpoint(const rmm::ModelPtr &model) {
  const int sid(model->pointedSibilingId());
  if (model->canUnpoint(sid)) {
    ROS_INFO_STREAM("Unpointing sibiling[" << sid << "] ...");
    model->unpoint(sid);
    return true;
  } else {
    return false;
  }
}

bool select(const rmm::ModelPtr &model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canSelect(sid)) {
    ROS_INFO_STREAM("Selecting sibiling[" << sid << "] ...");
    model->select(sid, /* allow_multi_selection = */ true);
    return true;
  } else {
    return false;
  }
}

bool deselect(const rmm::ModelPtr &model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canDeselect(sid)) {
    ROS_INFO_STREAM("Deselecting sibiling[" << sid << "] ...");
    model->deselect(sid);
    return true;
  } else {
    return false;
  }
}

bool descend(const rmm::ModelPtr &model) {
  const int sid(randomSid(model->currentLevel()->numSibilings()));
  if (model->canDescend(sid)) {
    ROS_INFO_STREAM("Descending from sibiling[" << sid << "] ...");
    model->descend(sid, /* allow_multi_selection = */ true);
    return true;
  } else {
    return false;
  }
}

bool ascend(const rmm::ModelPtr &model) {
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
  ros::init(argc, argv, "random_model_operation");
  ros::NodeHandle nh;

  // load a menu from param
  rmm::ModelPtr model(rmm::Model::fromParam("menu_description"));
  if (!model) {
    return 1;
  }
  ROS_INFO_STREAM("Model:\n" << model->toString());
  ROS_INFO_STREAM("State:\n" << model->exportState(ros::Time::now(), false));

  // perform random operations on the menu model
  typedef bool (*Operation)(const rmm::ModelPtr &);
  std::array< Operation, 6 > operations = {point, unpoint, select, deselect, descend, ascend};
  for (int i = 0; i < 20; ++i) {
    // shuffle operation order and try one by one until the first success
    std::shuffle(operations.begin(), operations.end(), randomEngine());
    for (const Operation op : operations) {
      if ((*op)(model)) {
        ROS_INFO_STREAM("Model:\n" << model->toString());
        ROS_INFO_STREAM("State:\n" << model->exportState(ros::Time::now(), true));
        break;
      }
    }
  }

  return 0;
}