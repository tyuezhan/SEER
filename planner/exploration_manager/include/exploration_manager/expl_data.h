#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <kr_tracker_msgs/Transition.h>
#include "kr_tracker_msgs/PolyTrackerAction.h"
#include <std_srvs/Trigger.h>



using std::vector;
using Eigen::Vector3d;

namespace bg_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<std::string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Eigen::Vector3d> start_poss;
  //@yuwei
  kr_tracker_msgs::PolyTrackerActionGoal newest_traj_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
  double estop_time_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<std::pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<vector<Vector3d>> frontiers_occ_pred_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<int> info_gains_;
  vector<int> topo_ids_;
  vector<double> costs_;
  vector<double> utilities_;
  vector<Vector3d> door_pos_;
  vector<double> door_yaws_;
  vector<Vector3d> global_tour_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
};

struct ExplorationParam {
  // params
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  std::string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  bool detect_semantics_;
};

}  // namespace bg_planner

#endif