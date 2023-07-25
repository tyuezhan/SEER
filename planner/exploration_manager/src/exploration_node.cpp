#include <ros/ros.h>
#include <exploration_manager/bg_exploration_fsm.h>

#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}

using namespace bg_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  bgExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
