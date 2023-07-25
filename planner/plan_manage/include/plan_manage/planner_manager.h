#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <path_searching/astar2.h>
#include <kino_searching/kino_acc_astar.h>
#include <kino_searching/kino_yaw.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>

//optimization related
#include <plan_manage/poly_opt.h>
#include <traj_utils/planning_visualization.h>
#include <plan_manage/sfc_generator.hpp>

namespace bg_planner {
// bg Planner Manager
// Key algorithms of mapping and planning are called

class bgPlannerManager {
  // SECTION stable
public:
  bgPlannerManager();
  ~bgPlannerManager();

  /* main planning interface */
  void initPlanModules(ros::NodeHandle& nh);

  bool checkGoalClearance(Eigen::Vector3d &goal, double distance);
  double checkTrajCollision(double& distance); //return the collision time, if <0, no collision
  double checkTrajCollision(min_jerk::Trajectory& traj);
  
  bool planYawExplore(const Eigen::Vector3d& start_yaw, double next_yaw);
  bool planYawNoExplore(const Eigen::Vector3d &start_yaw,
                        const double &end_yaw,
                        bool lookfwd,
                        const double& relax_time = 0);
  bool planExploreTraj(const Eigen::Vector3d& goal,
                       Eigen::MatrixXd &start_state,
                       double &h_yaw);

  bool emergencyStop(Eigen::MatrixXd &start_state, 
                     Eigen::Vector3d &yaw_vec);


  bool stay(Eigen::MatrixXd &start_state, 
            double dur);



  void setPlanVis(PlanningVisualization::Ptr &vis);
  void setFrontier(shared_ptr<FrontierFinder> &ff);

  PlanParameters pp_;
  LocalTrajData local_data_;
  EDTEnvironment::Ptr edt_environment_;

  //front-end planning
  std::unique_ptr<Astar> path_finder_;
  std::unique_ptr<KinoAccAstar> kinoacc_path_finder_;
  std::unique_ptr<KinoYaw> kino_yaw_finder_;
  double time_res_ = 0.1;

  //opt planning
  PolySolver poly_traj_solver_;
  PolyYawSolver poly_yaw_solver_;
  bool have_opt_path_ = false;
  double local_plan_horizon_ = 7;
  sfc_gen::SfcGenerator::Ptr sfc_gen_;


private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;
  unique_ptr<HeadingPlanner> heading_planner_;
  bool use_astar_path_, use_kino_acc_path_;

  PlanningVisualization::Ptr visualization_;


  void calcNextYaw(Eigen::Vector3d &next_yaw, 
                  Eigen::Vector3d &last_yaw, 
                  Eigen::Vector3d &dir, 
                  double dt);
  void refineEndYaw(double &start_yaw, double &end_yaw);


  template <typename T>
  bool kinoPlan(Eigen::MatrixXd &start_state,
                Eigen::MatrixXd &end_state,
                ros::Time plan_time,
                std::vector<Eigen::Vector3d> &kino_path,
                T &finder);

public:

  // !SECTION
  typedef shared_ptr<bgPlannerManager> Ptr;
};
}  // namespace bg_planner

#endif