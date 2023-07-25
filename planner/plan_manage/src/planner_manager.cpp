// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
namespace bg_planner
{
  // SECTION interfaces for setup and query

  bgPlannerManager::bgPlannerManager()
  {
  }

  bgPlannerManager::~bgPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void bgPlannerManager::initPlanModules(ros::NodeHandle &nh)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);

    nh.param("manager/use_astar_path", use_astar_path_, false);
    nh.param("manager/use_kino_acc_path", use_kino_acc_path_, false);
    nh.param("manager/time_res", time_res_, 0.1);

    nh.param("search/horizon", local_plan_horizon_, 7.0);

    /* optimization parameters */
    Eigen::VectorXd w_total(4), b_total(2), w_yaw_total(3), b_yaw_total(2);
    nh.param("optimization/w_time", w_total(0), 32.0);
    nh.param("optimization/w_vel", w_total(1), 128.0);
    nh.param("optimization/w_acc", w_total(2), 128.0);
    nh.param("optimization/w_sta_obs", w_total(3), 128.0);

    nh.param("optimization/w_refyaw", w_yaw_total(0), 2.0);
    nh.param("optimization/w_dyaw", w_yaw_total(1), 128.0);
    nh.param("optimization/w_ddyaw", w_yaw_total(2), 128.0);

    nh.param("manager/max_dyaw", pp_.max_dyaw_, -1.0);
    nh.param("manager/max_ddyaw", pp_.max_ddyaw_, -1.0);

    Vec3f map_size, map_origin;
    nh.param("manager/map_size_x", map_size(0), -1.0);
    nh.param("manager/map_size_y", map_size(1), -1.0);
    nh.param("manager/map_size_z", map_size(2), -1.0);

    nh.param("manager/map_origin_x", map_origin(0), -1.0);
    nh.param("manager/map_origin_y", map_origin(1), -1.0);
    nh.param("manager/map_origin_z", map_origin(2), -1.0);

    b_total << pp_.max_vel_, pp_.max_acc_;
    b_yaw_total << pp_.max_dyaw_, pp_.max_ddyaw_;
    
    pp_.max_dyaw_ -= 0.2;


    local_data_.traj_id_ = 0;
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    /*  local planner intial  */
    if (use_astar_path_)
    {
      path_finder_.reset(new Astar);
      // path_finder_->setParam(nh);
      // path_finder_->setEnvironment(edt_environment_);
      // path_finder_->init();
      path_finder_->init(nh, edt_environment_);
      //std::cout << "[PlannerManager]: use astar mode" << std::endl;
    }

    if (use_kino_acc_path_)
    {
      kinoacc_path_finder_.reset(new KinoAccAstar);
      // kinoacc_path_finder_->setParam(nh);
      kinoacc_path_finder_->init(nh, edt_environment_);
      //std::cout << "[PlannerManager]: use acc mode" << std::endl;
    }

    kino_yaw_finder_.reset(new KinoYaw);
    // kinoacc_path_finder_->setParam(nh);
    kino_yaw_finder_->init(nh);

    poly_traj_solver_.init(w_total, b_total);
    poly_yaw_solver_.init(w_yaw_total, b_yaw_total);
    
    //safe corridor setup
    sfc_gen_.reset(new sfc_gen::SfcGenerator);
    sfc_gen_->initROS(nh);
  }

  void bgPlannerManager::setPlanVis(PlanningVisualization::Ptr &vis)
  {
    visualization_ = vis;
  }


  void bgPlannerManager::setFrontier(shared_ptr<FrontierFinder> &ff)
  {
    kino_yaw_finder_->frontier_finder_ = ff;
  }


  bool bgPlannerManager::checkGoalClearance(Eigen::Vector3d &goal, double distance)
  {
    const double dr = 0.1;
    double dtheta = 30;
    double new_x, new_y;
    for (double r = 0.0; r <= distance + 1e-3; r += dr)
    {
      for (double theta = -90; theta <= 270; theta += dtheta)
      {
        new_x = goal(0) + r * cos(theta);
        new_y = goal(1) + r * sin(theta);
        Eigen::Vector3d new_pt(new_x, new_y, goal(2));
        if (sdf_map_->getInflateOccupancy(new_pt) == 1)
        {
          return false;
        }
      }
    }

    return true;
  }

  double bgPlannerManager::checkTrajCollision(double &distance)
  {

    double t_now = (ros::Time::now() - local_data_.start_time_).toSec();
    Eigen::Vector3d cur_pt = local_data_.traj_.getPos(t_now);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double check_time = t_now;
    // double t_2_3 = local_data_.duration_ > 1.0 ? local_data_.duration_ * 2 / 3 : local_data_.duration_;
    while (radius < 6.0 && check_time < local_data_.duration_)
    {
      fut_pt = local_data_.traj_.getPos(check_time);
      if (sdf_map_->getInflateOccupancy(fut_pt) == 1)
      {
        distance = radius;
        // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
        std::cout << "============= collision at: " << fut_pt.transpose() << std::endl;
        std::cout << "local_data_.start_time_ " << local_data_.start_time_ << std::endl;
        std::cout << " t_now " <<  t_now << "check_time  " << check_time  << std::endl;

        return check_time - t_now; // return the time to collide
      }
      radius = (fut_pt - cur_pt).norm();
      check_time += 0.02;
    }
    return -1.0;
  }


  double bgPlannerManager::checkTrajCollision(min_jerk::Trajectory& traj)
  {
    Eigen::Vector3d cur_pt = traj.getPos(0.0);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double check_time = 0.02;
    double dur = traj.getTotalDuration();
    // double t_2_3 = local_data_.duration_ > 1.0 ? local_data_.duration_ * 2 / 3 : local_data_.duration_;
    while (check_time < dur)
    {
      fut_pt = traj.getPos(check_time);
      if (sdf_map_->getInflateOccupancy(fut_pt) == 1)
      {
        // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
        std::cout << "============= collision at: " << fut_pt.transpose() << std::endl;


        return check_time; // return the time to collide
      }
      check_time += 0.02;
    }
    return -1.0;
  }



  // ============= trajectory planning
  bool bgPlannerManager::planExploreTraj(const Eigen::Vector3d &goal,
                                         Eigen::MatrixXd &start_state,
                                         double &h_yaw)
  { // 3 * 3  [pos, vel, acc]
    // end State could be obtained from previous planned trajs
    Eigen::MatrixXd end_state(3, 3); // include the start and end state
    Eigen::Vector3d local_target = goal;
    Eigen::MatrixXd inner_pts; // (3, N -1)
    Eigen::VectorXd allo_ts;
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<Eigen::Vector3d> path_pts;
    std::vector<double> path_psi;
    end_state.setZero();

    // step one: get the local target
    double dist = (goal - start_state.col(0)).norm();

    // std::cout << "[planExploreTraj]: local_target is " << local_target.transpose() << std::endl;
    // if (!have_opt_path_ || dist < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    // {
    //   end_state << local_target, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1);
    // }
    // else
    // {
    //   end_state << local_target, local_data_.traj_.getVel(local_data_.duration_), local_data_.traj_.getAcc(local_data_.duration_);
    // }


    end_state << local_target, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1);

    visualization_->displayGoalPoint(local_target, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);

    ros::Time time_now = ros::Time::now();
    // step two: kinodynamic path searching considering obstacles avoidance
    if (!kinoPlan(start_state, end_state, time_now, path_pts, kinoacc_path_finder_))
    {
      std::cout << "[planExploreTraj]: kinodynamic search fails!" << std::endl;
      
      path_finder_->reset();
      if (path_finder_->search(start_state.col(0), local_target) == Astar::REACH_END) 
      {
        std::cout << "[planExploreTraj]: A star search success" << std::endl;
        path_pts = path_finder_->getPath();
      }else{
        return false;
      }
    }
  
    if (path_pts.size() == 1)
    {
      stay(start_state, time_res_);
      return true;
    }
    
    end_state.col(0) = path_pts.back(); // to ensure the goal position is collision-free


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    sdf_map_->getPointCloud(*cloud_ptr);
    sfc_gen_->getPolyConst(path_pts, 
                           time_res_,
                           cloud_ptr, 
                           inner_pts, 
                           allo_ts, 
                           hPolys);
    sfc_gen_->visPoly();

    // update h_yaw
    auto dir = inner_pts.col(0) - start_state.col(0);
    h_yaw = atan2(dir(1), dir(0));

    // step four: unconstrained optimziation
    if (!poly_traj_solver_.minJerkTrajOpt(inner_pts,
                                          allo_ts,
                                          start_state,
                                          end_state,
                                          hPolys))
    {
      printf("\033[33m [planExploreTraj]: optimization fails! \n\033[0m");
      min_jerk::Trajectory fail_traj = poly_traj_solver_.getTraj();

      //visualization_->displayOptimalList(fail_traj.getConstPoints(5), Eigen::Vector4d(1, 0, 1, 1), 0);
      return false;
    }

    // step up local data
    have_opt_path_ = true;

    min_jerk::Trajectory cur_traj;
    poly_traj_solver_.getTraj(cur_traj);

    if (checkTrajCollision(cur_traj) > 0.0)
    {
       printf("\033[33m [planExploreTraj]: optimization fails! It collides with obstacles \n\033[0m");
      return false;
    }

    local_data_.traj_ = cur_traj;
    local_data_.duration_ = local_data_.traj_.getTotalDuration();
    local_data_.traj_id_ += 1;
    local_data_.start_time_ = time_now;

    // std::cout << "local_data_.duration_  " << local_data_.duration_  << std::endl;
    visualization_->displayKinoAStarList(path_pts, Eigen::Vector4d(0.8, 1, 0, 1), 0);
    visualization_->displayInitPathList(inner_pts, 0.1, 0);
    visualization_->displayOptimalList(local_data_.traj_.getConstPoints(5), Eigen::Vector4d(1, 0, 0, 1), 0);
    std::cout << "\n[planExploreTraj]: local planning success!" << std::endl;
    return true;


  }

  // ============= trajectory planning
  bool bgPlannerManager::emergencyStop(Eigen::MatrixXd &start_state,
                                       Eigen::Vector3d &yaw_vec)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Vector3d cur_vel = start_state.col(1);
    // std::cout << "\n[planExploreTraj]----------------------------- cur_vel / pp_.max_acc_ " << cur_vel / pp_.max_acc_<< std::endl;
    //t = v/a;
    Eigen::Vector3d T = (cur_vel /4.0).cwiseAbs();

    auto t_lb =  T.maxCoeff(); 
    std::cout << "\n[planExploreTraj]----------------------------- T" << T << std::endl;
    double stop_yaw_acc = 1.5 * pp_.max_dyaw_ ;
    double t_yaw = std::abs(yaw_vec(1)/stop_yaw_acc);

    double dur = std::max(t_lb, t_yaw);
    std::cout << "\n[planExploreTraj]----------------------------- dur " << dur << std::endl;
    
    stop_yaw_acc = -yaw_vec(1) / dur;
    Eigen::Vector3d stop_acc = -cur_vel/dur;
    Eigen::Vector3d stop_pos = start_state.col(0) + cur_vel * dur +  0.5 * stop_acc *dur * dur;
    std::cout << "\n[planExploreTraj]----------------------------- stop_acc " << stop_acc << std::endl;
    std::cout << "\n[planExploreTraj]----------------------------- stop_yaw_acc " << stop_yaw_acc << std::endl;


    std::cout << "\n[planExploreTraj]----------------------------- start_state " << start_state << std::endl;




    Eigen::Matrix<double, 3, 6> boundCond;
    boundCond.col(0) = start_state.col(0);
    boundCond.col(1) = cur_vel;
    boundCond.col(2) = stop_acc;
    boundCond.col(3) = stop_pos;
    boundCond.col(4) = ZERO;
    boundCond.col(5) = ZERO;

    std::vector<Eigen::Matrix<double, 3, 6>> bdConds;
    std::vector<double> durs;
    durs.push_back(dur);
    bdConds.push_back(boundCond);

    boundCond.col(0) = stop_pos;
    boundCond.col(1) = ZERO;
    boundCond.col(2) = ZERO;
    durs.push_back(0.001);
    bdConds.push_back(boundCond);



    local_data_.traj_ = min_jerk::Trajectory(bdConds, durs);
    local_data_.duration_ = local_data_.traj_.getTotalDuration();
    local_data_.traj_id_ += 1;

    std::cout << " local_data_.traj_.checkMaxVelRate " << local_data_.traj_.getMaxVelRate() << std::endl;
    std::cout << " minJerkTraj_.getMaxVelRate() " << local_data_.traj_.getMaxVelRate() << std::endl;


    double stop_yaw = yaw_vec(0) + yaw_vec(1) * dur +  0.5 * stop_yaw_acc *dur * dur;


    //for yaw trajectory 
    Eigen::Matrix<double,1, 6> boundCond_yaw;
    boundCond_yaw(0) = yaw_vec(0);
    boundCond_yaw(1) = yaw_vec(1);
    boundCond_yaw(2) = stop_yaw_acc;
    boundCond_yaw(3) = stop_yaw;
    boundCond_yaw(4) = 0.0;
    boundCond_yaw(5) = 0.0;



    std::vector<Eigen::Matrix<double, 1, 6>> bdConds_yaw;
    bdConds_yaw.push_back(boundCond_yaw);

    boundCond_yaw(0) = stop_yaw;
    boundCond_yaw(1) = 0.0;
    boundCond_yaw(2) = 0.0;
    bdConds_yaw.push_back(boundCond_yaw);


    local_data_.yaw_traj_ = min_yaw_jerk::Trajectory(bdConds_yaw, durs);
    local_data_.yaw_duration_ = local_data_.yaw_traj_.getTotalDuration();


    std::cout << "local_data_.duration_" << local_data_.duration_ << std::endl;
    std::cout << "local_data_.yaw_duration_" << local_data_.yaw_duration_ << std::endl;
    std::cout << "local_data_.yaw_traj_.getMaxVelRate() " << local_data_.yaw_traj_.getMaxVelRate() << std::endl;

    return true;
  }



  bool bgPlannerManager::stay(Eigen::MatrixXd &start_state,
                                       double dur)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix3d end_state;
    end_state << start_state.col(0) + 0.1 * start_state.col(1), ZERO, ZERO;

    //t = v/a;
    double t_lb = (start_state.col(1)).norm() / pp_.max_acc_; 
    
    dur = std::max(t_lb, dur);

    min_jerk::JerkOpt stopMJO;
    stopMJO.reset(start_state, end_state, 2);

    double ts = dur / 2;
    Eigen::Vector3d mid_point = 3.0/4.0 * start_state.col(0) + end_state.col(0) / 4.0;

    stopMJO.generate(mid_point, Eigen::Vector2d(ts, ts));
    std::cout << "\n[planExploreTraj]----------------------------- end_state " << end_state << std::endl;
    // update trajectory
    stopMJO.getTraj(local_data_.traj_);
    local_data_.duration_ = local_data_.traj_.getTotalDuration();
    local_data_.traj_id_ += 1;

    return true;
  }




  // ============= yaw planning
  bool bgPlannerManager::planYawExplore(const Eigen::Vector3d &start_yaw, double next_yaw)
  {
    if(local_data_.traj_.is_empty())
    {
      std::cout << "[localPlanner]: No trajectory now, canot plan yaw ! "  << std::endl;
      return false;
    }

    min_jerk::Trajectory temp_traj;
    temp_traj = local_data_.traj_;

    Eigen::MatrixXd yawState(2, 3);
    yawState.setZero();
    yawState.row(0) = start_yaw;
    yawState(1, 0) = next_yaw;
    kino_yaw_finder_->reset();

    if (!kino_yaw_finder_->yawSearch(yawState, temp_traj))
    {
      std::cout << "[planExploreTraj]: kinodynamic search fails!" << std::endl;
      return false;
    }

    Eigen::MatrixXd inner_yaws;
    Eigen::VectorXd durs;
    kino_yaw_finder_->getYawInners(inner_yaws, durs);

    if (!poly_yaw_solver_.minJerkYawOpt(inner_yaws,
                                        durs,
                                        yawState,
                                        true))
    {
      printf("\033[33m[localPlanner]: yaw optimization fails! \n\033[0m");
      return false;
    }

    poly_yaw_solver_.getTraj(local_data_.yaw_traj_);
    local_data_.yaw_duration_ = local_data_.yaw_traj_.getTotalDuration();
    return true;
  }

  // ============= yaw fit
  bool bgPlannerManager::planYawNoExplore(const Eigen::Vector3d &start_yaw,
                                          const double &end_yaw,
                                          bool lookfwd,
                                          const double &relax_time)
  {

    if(local_data_.traj_.is_empty())
    {
      std::cout << "[localPlanner]: No trajectory now, canot plan yaw ! "  << std::endl;
      return false;
    }

    const int seg_num = std::max(local_data_.traj_.getPieceNum(), 8);
    double total_time = local_data_.duration_  + relax_time;
    double dt_yaw = total_time/ seg_num; // time of B-spline segment

    Eigen::MatrixXd inner_yaws(1, seg_num - 1);
    Eigen::VectorXd durs = dt_yaw * Eigen::VectorXd::Ones(seg_num);
    inner_yaws.setZero();
    Eigen::MatrixXd yawState(2, 3);
    yawState.setZero();
    yawState.row(0) = start_yaw;

    Eigen::Vector3d pc, pf, pd;

    if (lookfwd)
    {
      // std::cout << "[localPlanner]: lookfwd !" << std::endl;
      Eigen::Vector3d last_yaw3d = start_yaw;
      Eigen::Vector3d waypt;
      double forward_t = 1.0;

      for (int i = 0; i < seg_num; ++i)
      {
        double tc = i * dt_yaw;

        pc = local_data_.traj_.getPos(tc);
        double tf = min(local_data_.duration_, tc + forward_t);
        pf = local_data_.traj_.getPos(tf);
        pd = pf - pc;

        calcNextYaw(waypt, last_yaw3d, pd, dt_yaw);
        //std::cout << "[localPlanner]:  waypt is " <<  waypt << std::endl;
        if (i < seg_num -1)
        {
          last_yaw3d = waypt;
          inner_yaws(0, i) =  waypt(0);
        }else{
          yawState.row(1) = waypt;
        }

      }

    }else{


      double new_end_yaw = end_yaw;
      double init_start_yaw = start_yaw(0);

      refineEndYaw(init_start_yaw, new_end_yaw);

      double init_dyaw = (new_end_yaw - start_yaw(0)) / total_time;

      while (init_dyaw < -M_PI)
        init_dyaw += 2 * M_PI;
      while (init_dyaw >= M_PI)
        init_dyaw -= 2 * M_PI;

      // clip the yaw dot
      if (init_dyaw > pp_.max_dyaw_)
      {
        init_dyaw = pp_.max_dyaw_;
      }
      else if (init_dyaw < -pp_.max_dyaw_)
      {
        init_dyaw = -pp_.max_dyaw_;
      }
     
      Eigen::Vector2d start_state, end_state;
      start_state << start_yaw(0), start_yaw(1);
      end_state << start_yaw(0) + init_dyaw * seg_num * dt_yaw, 0.0;

      kino_yaw_finder_->computeShotTraj(start_state, end_state, total_time);
      kino_yaw_finder_->getShotTraj(dt_yaw, inner_yaws);

      //yawState(1, 0) = new_end_yaw;
      yawState(1, 0) = end_state(0);

    }

    if (!poly_yaw_solver_.minJerkYawOpt(inner_yaws,
                                        durs,
                                        yawState,
                                        false))
    {
      printf("\033[33m[localPlanner]: yaw optimization fails! \n\033[0m");

      if(!local_data_.yaw_traj_.is_empty())
      {
        local_data_.yaw_traj_.clear();
      }
      return false;
    }

    poly_yaw_solver_.getTraj(local_data_.yaw_traj_);
    local_data_.yaw_duration_ = local_data_.yaw_traj_.getTotalDuration();
    return true;
  }


  void bgPlannerManager::refineEndYaw(double &start_yaw, double &end_yaw)

  {
    //range the angle into [-PI, PI)
    if (end_yaw <= 0)
    {
      while ( end_yaw - start_yaw< -M_PI)
        end_yaw += 2 * M_PI;

    }else if (end_yaw > 0)
    {
      while (end_yaw - start_yaw >= M_PI)
      end_yaw -= 2 * M_PI;
    }

    std::cout << "[localPlanner]:end_yaw" << end_yaw << std::endl;
    double diff = fabs(end_yaw - start_yaw);
    if (diff >= 2 * M_PI - diff)
    {
      if (end_yaw <= 0)
      { 
        if(start_yaw >= 0)
        {
          end_yaw += 2 * M_PI;
        }
      }
      if (end_yaw >= 0)
      {
        if (start_yaw <= 0)
        {
          end_yaw -= 2 * M_PI;
        }
        
      }
    }
  }


  void bgPlannerManager::calcNextYaw(Eigen::Vector3d &next_yaw, 
                                     Eigen::Vector3d &last_yaw, 
                                     Eigen::Vector3d &dir, 
                                     double dt)
  {



    double yaw_temp = dir.norm() > 0.1
                    ? atan2(dir(1), dir(0))
                    : last_yaw(0);
    //std::cout << " yaw_temp is  " << yaw_temp << std::endl;


    double diff = fabs(yaw_temp - last_yaw(0));
    if (diff >= 2 * M_PI - diff)
    {
      if (yaw_temp <= 0 && last_yaw(0) >= 0)
      {
        yaw_temp += 2 * M_PI;
      }
      if (yaw_temp >= 0 && last_yaw(0) <= 0)
      {
        yaw_temp -= 2 * M_PI;
      }
    }

    double d_yaw = (yaw_temp - last_yaw(0))/dt;


    if (d_yaw >= M_PI)
    {
      d_yaw -= 2 * M_PI;
    }
    if (d_yaw < -M_PI)
    {
      d_yaw += 2 * M_PI;
    }

    // clip the yaw dot
    if (d_yaw > pp_.max_dyaw_)
    {
      d_yaw = pp_.max_dyaw_;
    }
    else if (d_yaw < -pp_.max_dyaw_)
    {
      d_yaw= -pp_.max_dyaw_;
    }

    next_yaw <<  last_yaw(0) + d_yaw * dt, d_yaw,  0.0;

    return;
  }

  // use kinodynamic a* to generate a path and get hpoly
  template <typename T>
  bool bgPlannerManager::kinoPlan(Eigen::MatrixXd &start_state,
                                  Eigen::MatrixXd &end_state,
                                  ros::Time plan_time,
                                  std::vector<Eigen::Vector3d> &kino_path,
                                  T &finder)
  {

    kino_path.clear();
    finder->reset();

    int status = finder->search(start_state, end_state, plan_time, false);

    if (status == KINO_SEARCH_RESULT::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;
      // retry searching with discontinuous initial state
      finder->reset();
      status = finder->search(start_state, end_state, plan_time, false);
      if (status == KINO_SEARCH_RESULT::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }
    finder->getKinoTraj(time_res_, kino_path);

    if ( (kino_path.back() - end_state.col(0)).norm() > 1.0)
    {
      end_state.col(0) = kino_path.back();
    }

    return true;
  }

} // namespace bg_planner
