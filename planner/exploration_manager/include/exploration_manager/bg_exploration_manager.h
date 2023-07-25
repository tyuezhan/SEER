#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <traj_utils/planning_visualization.h>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace bg_planner {
class EDTEnvironment;
class SDFMap;
class bgPlannerManager;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, ROTATE};

class bgExplorationManager {
public:
  bgExplorationManager();
  ~bgExplorationManager();

  void initialize(ros::NodeHandle& nh);


  int planExploreMotionBC(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, 
                          const Vector3d& yaw);


  int computeFrontierUtilities(const Vector3d& cur_pos, const Vector3d& vel, const Vector3d& cur_yaw, 
                                const vector<Eigen::Vector3d>& points, const vector<double>& yaws,
                                const vector<int>& info_gains, vector<double>& costs, vector<double>& utilities);
  int planBC(const Vector3d& next_pos, const double& next_yaw, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);
 
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<bgPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  // unique_ptr<ViewFinder> view_finder_;
  visualization_msgs::Marker mk;
  void getReplanGoal(Vector3d &replan_next_pos, double &replan_next_yaw)
  {
    replan_next_pos = replan_next_pos_;
    replan_next_yaw  = replan_next_yaw_;
  }

  void setVis(shared_ptr<PlanningVisualization> &vis);
  int plan_fail_cnt_ = 0;

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                      vector<int>& indices);

  void shortenPath(vector<Vector3d>& path);

  ros::Publisher next_goal_pub_; 
  Vector3d replan_next_pos_;
  double replan_next_yaw_;
  bool is_planning_ = false;



  double radius_far_    = 7.0;
  double radius_close_  = 2.0;
  double radius_noplan_ = 0.5;
  bool terminate_condition_vol_enabled_;
  double terminate_condition_vol_;
  double v_fov_;
public:
  typedef shared_ptr<bgExplorationManager> Ptr;
};

}  // namespace bg_planner

#endif