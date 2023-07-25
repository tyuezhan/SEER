#ifndef _KINO_YAW_H
#define _KINO_YAW_H

#include <ros/console.h>
#include <ros/ros.h>
#include <kino_searching/kino_utils.hpp>
#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>


namespace bg_planner {

class KinoYaw {
private:

  std::vector<YawNodePtr>  yaw_nodes_;
  double max_dyaw_  = 0.8;
  double max_ddyaw_ = 0.2;
  double yaw_resolution_ = 0.05;

  // parameters
  double w_time_ = 3.0; 
  double w_uyaw_ = 5.0;
  double w_ref_ = 5.0;
  double w_semantics_ = 1.0;
  double w_info_ = 0.001;

  int vis_check_num_ = 4;
  double half_fov_theta_ = 2.0;

  int yawToIndex(double yaw);
  Eigen::VectorXd durations_;


  Eigen::VectorXd coef_shot_;
  double t_shot_ = 0.0;
  bool is_shot_succ_ = false;

  int getInfoPenalty(const Eigen::Vector3d &ego_pos, 
                        double &ego_yaw);
  
  /* perception value computation*/
  double getSemanticsPenalty(const Eigen::Vector3d &ego_pos, 
                              double &ego_yaw,
                              double &heading_yaw);

  inline double small_abs(double angle){
    // range the angle into (-PI, PI]

    double psi_abs = abs(angle);
    double psi = std::min(psi_abs,   2*M_PI - psi_abs);
    return psi;
  }

  std::vector<Eigen::Vector3d> door_pos;
  std::vector<double> door_yaws;


public:
  KinoYaw(){};
  ~KinoYaw(){};

  void reset();
  std::shared_ptr<FrontierFinder> frontier_finder_;

  bool computeShotTraj(Eigen::Vector2d state1, 
                       Eigen::Vector2d state2,
                       double time_to_goal);
  void getShotTraj(double &delta_t,
                   Eigen::MatrixXd &inner_yaws);

  void init(ros::NodeHandle& nh);

  bool yawSearch(Eigen::MatrixXd &yaw_state,
                 min_jerk::Trajectory &traj);


  void getYawInners(Eigen::MatrixXd &yaw_list, 
                    Eigen::VectorXd &durs);
             
  typedef std::shared_ptr<KinoYaw> Ptr;

};

}  // namespace 

#endif
