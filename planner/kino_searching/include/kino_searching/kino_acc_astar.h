#ifndef _KINO_ACC_ASTAR_H
#define _KINO_ACC_ASTAR_H


#include <ros/console.h>
#include <ros/ros.h>
#include <kino_searching/kino_utils.hpp>
#include "plan_env/edt_environment.h"

namespace bg_planner {

class KinoAccAstar {
private:
  /* ---------- main data structure ---------- */
  vector<AccNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable<AccNodePtr> expanded_nodes_; // the candidate nodes 
  std::priority_queue<AccNodePtr, std::vector<AccNodePtr>, NodeComparator<AccNodePtr>> open_set_;
  std::vector<AccNodePtr> path_nodes_;
  /* ---------- record data ---------- */
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, end_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  Eigen::MatrixXd coef_shot_;
  vector<Eigen::Vector3d> acc_inputs_;
  double t_shot_ = 0.0;
  bool is_shot_succ_ = false;
  bool has_path_ = false;
  KinoSearchParameters ksp_;
  double max_search_time_ = 1.0;
  /* map */
  EDTEnvironment::Ptr edt_env_;

  /* shot trajectory */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);

public:
  KinoAccAstar(){};
  ~KinoAccAstar();

  /* main API */
  void init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env);
  void reset();
  void getKinoTraj(double delta_t, std::vector<Eigen::Vector3d> &path_list);
  void getKinoTraj(double delta_t, 
                   Eigen::MatrixXd &endState,
                   std::vector<Eigen::Vector3d> &path_list);

  int search(Eigen::MatrixXd startState, 
             Eigen::MatrixXd endState, 
             ros::Time time_start, 
             bool init);

  typedef shared_ptr<KinoAccAstar> Ptr;
};

}  // namespace 

#endif
