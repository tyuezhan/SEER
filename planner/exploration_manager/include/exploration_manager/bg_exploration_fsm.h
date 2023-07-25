#ifndef _BG_EXPLORATION_FSM_H_
#define _BG_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace bg_planner {
class bgPlannerManager;
class bgExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { 
  INIT, 
  WAIT_TRIGGER, 
  INIT_ROTATE,
  PLAN_BEHAVIOR, 
  REPLAN_TRAJ, 
  EXEC_TRAJ, 
  FINISH, 
  EMERGENCY_STOP};

class bgExplorationFSM {
private:
  /* planning utils */
  shared_ptr<bgPlannerManager> planner_manager_;
  shared_ptr<bgExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool flag_escape_emergency_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, traj_goal_pub_;
  std::string srv_name_;
  
  ros::Time odom_time_, plan_stime_, behavior_stime_;
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void publishTraj();
  void updateStartState(double delay = 0.01);
  Vector3d replan_next_pos_;
  double replan_next_yaw_;
  bool has_traj_ = false;
  double priodic_time_ = 0.0;

  inline double range(double angle){
    // range the angle into [-PI, PI)
    double psi = angle;

    while (psi < -M_PI)
      psi += 2 * M_PI;
    while (psi >= M_PI)
      psi -= 2 * M_PI;

    return psi;
  }

  std::vector<double> init_target_yaw_;
  double cur_target_yaw_;
  Eigen::Vector3d cur_target_pos_;
  bool set_init_yaw_ = false;


public:
  bgExplorationFSM(/* args */) {
  }
  ~bgExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace

#endif