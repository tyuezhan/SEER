
#include <plan_manage/planner_manager.h>
#include <exploration_manager/bg_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/bg_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace bg_planner
{
  void bgExplorationFSM::init(ros::NodeHandle &nh)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /*  Fsm param  */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/estop_time",  fp_->estop_time_, -1.0);

    nh.param("srv_name", srv_name_, std::string(" "));

    /* Initialize main modules */
    expl_manager_.reset(new bgExplorationManager);
    expl_manager_->initialize(nh);
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_ = expl_manager_->planner_manager_;
    expl_manager_->setVis(visualization_);

    state_ = EXPL_STATE::INIT;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "INIT_ROTATE", "PLAN_BEHAVIOR", "REPLAN_TRAJ", "EXEC_TRAJ", "FINISH", "EMERGENCY_STOP"};
    fd_->static_state_ = true;
    fd_->trigger_ = false;
    fd_->start_acc_.setZero();

    /* Ros sub, pub and timer */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &bgExplorationFSM::FSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &bgExplorationFSM::safetyCallback, this);
    frontier_timer_ = nh.createTimer(ros::Duration(0.5), &bgExplorationFSM::frontierCallback, this);

    trigger_sub_ = nh.subscribe("waypoints", 1, &bgExplorationFSM::triggerCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &bgExplorationFSM::odometryCallback, this);

    traj_goal_pub_ = nh.advertise<kr_tracker_msgs::PolyTrackerActionGoal>("tracker_cmd", 10);

    std::cout << "[bgExplorationFSM::init] finished" << std::endl;
  }

  void bgExplorationFSM::FSMCallback(const ros::TimerEvent &e)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

    switch (state_)
    {
    case INIT:
    {
      // Wait for odometry ready
      if (!fd_->have_odom_)
      {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER:
    {
      // Do nothing but wait for trigger
      // ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case INIT_ROTATE:
    {

      static int fsm_num = 0;
      fsm_num++;

      double dyaw = range(cur_target_yaw_ - fd_->odom_yaw_);

      if (abs(dyaw) < 0.6 || !set_init_yaw_ || fsm_num == 200)
      {
        //trigger next rotation
        if (init_target_yaw_.size() > 0)
        { 

          updateStartState();
          cur_target_yaw_ = init_target_yaw_.back();
          expl_manager_->plan_fail_cnt_ = 0;
          int res = expl_manager_->planBC(cur_target_pos_, cur_target_yaw_,
                                          fd_->start_pt_,
                                          fd_->start_vel_,
                                          fd_->start_acc_,
                                          fd_->start_yaw_);
          publishTraj();
          set_init_yaw_ = true;
          init_target_yaw_.pop_back();

          fsm_num = 0;
        }else{
          transitState(PLAN_BEHAVIOR, "FSM");
          init_target_yaw_.clear();
          set_init_yaw_ = false;
        }
      }

      break;
    }
    case FINISH:
    {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }
    case PLAN_BEHAVIOR:
    {
      updateStartState();
      behavior_stime_ = plan_stime_;
      int res = callExplorationPlanner();
      if (res == SUCCEED)
      {
        publishTraj();
        transitState(EXEC_TRAJ, "FSM");
      }
      else if (res == NO_FRONTIER)
      {
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
      }
      else //  || res == FAIL or ROTATE
      {
        transitState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case REPLAN_TRAJ:
    {

      updateStartState();

      if ((replan_next_pos_ - fd_->start_pt_).norm() < 0.2 && abs(replan_next_yaw_-fd_->start_yaw_(0)) < 0.2 ){
        transitState(PLAN_BEHAVIOR, "FSM");
        break;
      }

      int res = expl_manager_->planBC(replan_next_pos_, 
                                      replan_next_yaw_,
                                      fd_->start_pt_,
                                      fd_->start_vel_,
                                      fd_->start_acc_,
                                      fd_->start_yaw_);

      if (res == SUCCEED)
      {
        // 1. publish the trajectory
        publishTraj();
        // 2. set the emer as true
        flag_escape_emergency_ = true;
        // 3. transite to exec
        transitState(EXEC_TRAJ, "FSM");
      }
      else if (res == FAIL)
      {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail, plan behavior");
        fd_->static_state_ = true;
        transitState(PLAN_BEHAVIOR, "FSM");
      }else if (res == ROTATE)
      {
        ROS_WARN("try to rotate first");

        init_target_yaw_.push_back(fd_->start_yaw_(0) - M_PI + 0.1);
        cur_target_yaw_ = fd_->start_yaw_(0) - M_PI + 0.1;
        cur_target_pos_ = fd_->start_pt_;
        cur_target_pos_(2) += 0.01;
        cur_target_pos_(0) += 0.02;
        transitState(INIT_ROTATE, "FSM");
      }
      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->traj_.getPos(t_cur);

      if ((fd_->odom_pos_- pos).norm() > 1.0)
      {
        ROS_WARN("Replan: the agent does not follow the trajectory, stop!");
        cout << "fd_->odom_pos_" << fd_->odom_pos_ << endl;
        cout << "pos" << pos << endl;
        has_traj_ = false;
        transitState(EMERGENCY_STOP, "FSM");

      }
      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;

      bool fc = expl_manager_->frontier_finder_->isFrontierCovered();

      if (time_to_end < fp_->replan_thresh1_)
      {
        ROS_WARN("Replan: traj fully executed, plan behaviors =================================");
        has_traj_ = false;
        transitState(PLAN_BEHAVIOR, "FSM");
        return;
      }

      // Replan if next frontier to be visited is covered
      double t_bcur = (ros::Time::now() - behavior_stime_).toSec();  

      if (t_bcur > fp_->replan_thresh3_ && fc)
      {
        ROS_WARN("Replan: cluster covered, plan behaviors =====================================");
        transitState(PLAN_BEHAVIOR, "FSM");
        return;
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_)
      {
        updateStartState(0.00);
        ROS_WARN("Emergency Stop =======================================");
        Eigen::MatrixXd start_state(3,3); // include the start and end state
        start_state << fd_->odom_pos_,
                       fd_->odom_vel_,
                       Eigen::Vector3d::Zero();

        planner_manager_->emergencyStop(start_state, fd_->start_yaw_);
        publishTraj();
      }
      else
      {
        if (fd_->odom_vel_.norm() <= 0.1)
        {
          std::cout << " fd_->odom_vel_ " << fd_->odom_vel_ << std::endl;
          fd_->static_state_ = true;
          transitState(PLAN_BEHAVIOR, "FSM");
        }

      }
      flag_escape_emergency_ = false;
      break;
    }
    }
  }

  void bgExplorationFSM::updateStartState(double delay)
  {
    plan_stime_ = odom_time_ + ros::Duration(delay);

    if (fd_->static_state_)
    {
      // Plan from static state (hover)
      fd_->start_pt_ = fd_->odom_pos_;
      fd_->start_vel_ = fd_->odom_vel_;
      fd_->start_acc_.setZero();

      fd_->start_yaw_(0) = fd_->odom_yaw_;
      fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
    }
    else
    {
      // Replan from non-static state, starting from 'replan_time' seconds later
      LocalTrajData *info = &planner_manager_->local_data_;
      // incase the t_r is larger than its original index
      // double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
      double t_r = std::min((plan_stime_ - info->start_time_).toSec(), info->duration_);
  
      fd_->start_pt_ = info->traj_.getPos(t_r);
      fd_->start_vel_ = info->traj_.getVel(t_r);
      fd_->start_acc_ = info->traj_.getAcc(t_r);

      if(!info->yaw_traj_.is_empty())
      {
        double yt_r = std::min((plan_stime_ - info->start_time_).toSec(), info->yaw_duration_);

        fd_->start_yaw_(0) = info->yaw_traj_.getPos(yt_r);
        fd_->start_yaw_(1) = info->yaw_traj_.getVel(yt_r);
        fd_->start_yaw_(2) = range(info->yaw_traj_.getAcc(yt_r));
      }else{
        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      }

    }
  }

  int bgExplorationFSM::callExplorationPlanner()
  {
    int res;
    res = expl_manager_->planExploreMotionBC(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
    expl_manager_->getReplanGoal(replan_next_pos_, replan_next_yaw_);
    return res;
  }

  void bgExplorationFSM::publishTraj()
  {
    // get the trajectory
    auto info = &planner_manager_->local_data_;
    info->start_time_ = plan_stime_;
    std::cout << "bgExplorationFSM::publishTraj plan_stime_ is " << plan_stime_ <<  std::endl;
    // publish the message for multi-agent connections and for trajectory server
    kr_tracker_msgs::PolyTrackerActionGoal traj_act_msg;
    int order = 5;

    traj_act_msg.goal.order = order;
    traj_act_msg.goal.set_yaw = false;
    Eigen::VectorXd durs = info->traj_.getDurations();
    int piece_num = info->traj_.getPieceNum();

    traj_act_msg.goal.t_start = info->start_time_;
    traj_act_msg.goal.seg_x.resize(piece_num);
    traj_act_msg.goal.seg_y.resize(piece_num);
    traj_act_msg.goal.seg_z.resize(piece_num);


    for (int i = 0; i < piece_num; ++i)
    {
      min_jerk::CoefficientMat coeff = info->traj_[i].getCoeffMat(true);

      for (uint c = 0; c <= order; c++) {
        traj_act_msg.goal.seg_x[i].coeffs.push_back(coeff(0,order-c));
        traj_act_msg.goal.seg_y[i].coeffs.push_back(coeff(1,order-c));
        traj_act_msg.goal.seg_z[i].coeffs.push_back(coeff(2,order-c));
      }
      traj_act_msg.goal.seg_x[i].dt = durs[i];
      traj_act_msg.goal.seg_x[i].degree = order;

    }

    std::cout << "[ReplanFSM::publishTraj] durs is " << durs << std::endl;

    if(!info->yaw_traj_.is_empty())
    {
      int piece_num_yaw = info->yaw_traj_.getPieceNum();

      Eigen::VectorXd durs_yaw = info->yaw_traj_.getDurations();

      traj_act_msg.goal.seg_yaw.resize(piece_num_yaw);

      for (int i = 0; i < piece_num_yaw; ++i)
      {
        min_yaw_jerk::CoefficientMat coeff_yaw = info->yaw_traj_[i].getCoeffMat(true);

        for (uint c = 0; c <= order; c++) {
          traj_act_msg.goal.seg_yaw[i].coeffs.push_back(coeff_yaw(0,order-c));
        }
        traj_act_msg.goal.seg_yaw[i].dt = durs_yaw[i];
        traj_act_msg.goal.seg_yaw[i].degree = order;

      }
    }
    traj_act_msg.goal.separate_yaw = true;
    // publish the trajectory
    traj_goal_pub_.publish(traj_act_msg);
    std_srvs::Trigger trg;
    ros::service::call(srv_name_, trg);

    fd_->static_state_ = false;

    thread vis_thread(&bgExplorationFSM::visualize, this);
    vis_thread.detach();

    has_traj_ = true;
    priodic_time_ = 0.0;
  }

  void bgExplorationFSM::visualize()
  {
    auto info = &planner_manager_->local_data_;
    auto ed_ptr = expl_manager_->ed_;

    // Draw frontier
    static int last_ftr_num = 0;
    static int last_ftr_occ_pred_num = 0;
    for (unsigned int i = 0; i < ed_ptr->frontiers_.size(); ++i)
    {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.05,
                                visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
      //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    {
      visualization_->drawCubes({}, 0.05, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // Draw prediction at frontier clusters
    for (unsigned int i = 0; i < ed_ptr->frontiers_occ_pred_.size(); ++i)
    {
      vector<Eigen::Vector3d> tmp;
      for (auto p : ed_ptr->frontiers_occ_pred_[i]) {
        if (expl_manager_->frontier_finder_->occSearch(p) == 1) continue;
        tmp.push_back(p);
      }
      visualization_->drawCubes(tmp, 
                                0.065, Vector4d(0.5, 0.5, 0.5, 0.6),
                                // 0.05, visualization_->getColor(double(i) / ed_ptr->frontiers_occ_pred_.size(), 0.4),
                                "occ_prediction", i, 7);
    }
    for (int i = ed_ptr->frontiers_occ_pred_.size(); i < last_ftr_occ_pred_num; ++i)
    {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "occ_prediction", i, 7);
    }
    last_ftr_occ_pred_num = ed_ptr->frontiers_occ_pred_.size();

    // Draw door
    ROS_INFO("Draw arrows");
    visualization_->drawArrows(ed_ptr->door_pos_, ed_ptr->door_yaws_, 0.5,
                                {1, 0, 0, 1}, "doors", 0, 8);
  
  }

  void bgExplorationFSM::frontierCallback(const ros::TimerEvent &e)
  {
    static int delay = 0;
    if (++delay < 5)
      return;

    if (state_ == WAIT_TRIGGER || state_ == FINISH)
    {
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      ft->searchFrontiersBC();
      ft->predOnFrontiers();
      ft->computeFrontiersToVisitBC();
      ft->getFrontiers(ed->frontiers_);
      ft->getFrontierBoxes(ed->frontier_boxes_);
      ft->getFrontierPred(ed->frontiers_occ_pred_);

      // Draw frontier and bounding box
      for (unsigned int i = 0; i < ed->frontiers_.size(); ++i)
      {
        visualization_->drawCubes(ed->frontiers_[i], 0.05,
                                  visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                  "frontier", i, 4);
        // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
        // Vector4d(0.5, 0, 1, 0.3),
        //                         "frontier_boxes", i, 4);
      }
      for (int i = ed->frontiers_.size(); i < 50; ++i)
      {
        visualization_->drawCubes({}, 0.05, Vector4d(0, 0, 0, 1), "frontier", i, 4);
        // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
        // "frontier_boxes", i, 4);
      }

      // Draw prediction at frontier clusters
      for (unsigned int i = 0; i < ed->frontiers_occ_pred_.size(); ++i)
      {
        vector<Eigen::Vector3d> tmp;
        for (auto p : ed->frontiers_occ_pred_[i]) {
          if (expl_manager_->frontier_finder_->occSearch(p) == 1) continue;
          tmp.push_back(p);
        }
        visualization_->drawCubes(tmp, 
                                  0.1, Vector4d(0.5, 0.5, 0.5, 0.8),
                                  // 0.05, visualization_->getColor(double(i) / ed->frontiers_occ_pred_.size(), 0.4),
                                  "occ_prediction", i, 7);
      }
      for (unsigned int i = ed->frontiers_occ_pred_.size(); i < 50; ++i)
      {
        visualization_->drawCubes({}, 0.05, Vector4d(0, 0, 0, 1), "occ_prediction", i, 7);
      }
      
    }


  }

  void bgExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;
    if (state_ != WAIT_TRIGGER)
      return;
    fd_->trigger_ = true;
    cout << "Triggered!" << endl;
    transitState(PLAN_BEHAVIOR, "triggerCallback");
  }

  void bgExplorationFSM::safetyCallback(const ros::TimerEvent &e)
  {
    if (state_ == EXPL_STATE::EXEC_TRAJ)
    {
      // Check safety and trigger replan if necessary
      /***     Checking the trajectory  ***/
      double dist = 0.25;
      double ctime = planner_manager_->checkTrajCollision(dist);

      if (ctime < 0.0)
      {
        return;
      }
      if (ctime <= fp_->estop_time_) // as emergency time
      {
        ROS_WARN("safetyCallback: emergency stop! collision time is %f", ctime);
        transitState(EMERGENCY_STOP, "SAFETY");
      }
      else
      {
        ROS_WARN("safetyCallback: trajectory collision detected, replan behavior and trajectory ==============================");
        transitState(PLAN_BEHAVIOR, "safetyCallback");
      }
    }

    if (state_ == EXPL_STATE::EXEC_TRAJ)
    { 
      if (planner_manager_->checkGoalClearance(replan_next_pos_, 0.2))
      {
        //std::cout << "try to find a max distance goal around " << std::endl;
        /* try to find a max distance goal around */
        ROS_WARN("safetyCallback: goal collision detected, replan behavior and trajectory ==================================");
        transitState(PLAN_BEHAVIOR, "safetyCallback");
        return;
      }
    }
 
  }

  void bgExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {

    odom_time_ = msg->header.stamp;
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;

    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

    fd_->have_odom_ = true;

  }

  void bgExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
  {
    int pre_s = int(state_);
    state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
         << endl;
  }
} // namespace bg_planner
