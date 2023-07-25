#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <traj_utils/traj_min_jerk.hpp>
#include <traj_utils/traj_min_yaw.hpp>

using std::vector;

namespace bg_planner
{
  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_, max_dyaw_, max_ddyaw_; // physical limits

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;

    /* agent information */
    int mav_id_;
    double  mav_radius_;
  
  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;
    double duration_;
    ros::Time start_time_;
    //Eigen::Vector3d start_pos_;
    min_jerk::Trajectory traj_;

    /* for perception aware yaw planning*/
    //double start_yaw_;

    min_yaw_jerk::Trajectory yaw_traj_;
    double yaw_duration_;
  };

} // namespace

#endif