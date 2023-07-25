// #include <fstream>
#include <exploration_manager/bg_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace bg_planner
{
  // SECTION interfaces for setup and query

  bgExplorationManager::bgExplorationManager()
  {
  }

  bgExplorationManager::~bgExplorationManager()
  {
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
  }

  void bgExplorationManager::initialize(ros::NodeHandle &nh)
  {
    planner_manager_.reset(new bgPlannerManager);
    planner_manager_->initPlanModules(nh);

    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_ = edt_environment_->sdf_map_;
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));

    planner_manager_->setFrontier(frontier_finder_);

    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);

    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    nh.param("exploration/detect_semantics", ep_->detect_semantics_, false);

    nh.param("exploration/vm", ViewNode::vm_, -1.0);
    nh.param("exploration/am", ViewNode::am_, -1.0);
    nh.param("exploration/yd", ViewNode::yd_, -1.0);
    nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
    nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

    nh.param("exploration/radius_far", radius_far_, -1.0);
    nh.param("exploration/radius_close", radius_close_, -1.0);
    nh.param("exploration/radius_noplan", radius_noplan_, -1.0);
    
    nh.param("exploration/terminate_condition_vol_enabled", terminate_condition_vol_enabled_, false);
    nh.param("exploration/terminate_condition_vol", terminate_condition_vol_, -1.0);
    
    nh.param("frontier/v_fov", v_fov_, -1.0);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(nh, edt_environment_);
    ViewNode::map_ = sdf_map_;

    double resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    // Visualization for planner
    next_goal_pub_ = nh.advertise<visualization_msgs::Marker>("/behavior_goal_vis/next_goal", 100);

    mk.id = 1;
    plan_fail_cnt_ = 0;
  }

  void bgExplorationManager::setVis(shared_ptr<PlanningVisualization> &vis)
  {

    planner_manager_->setPlanVis(vis);
  }

  int bgExplorationManager::planExploreMotionBC(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
  {
    ros::Time t1 = ros::Time::now();
    auto t2 = t1;
    double frontier_time;

    // If volume check enabled
    if (terminate_condition_vol_enabled_) {
      if (edt_environment_->sdf_map_->getKnownVolume() > terminate_condition_vol_) {
        ROS_WARN_STREAM("Known Vol: " << edt_environment_->sdf_map_->getKnownVolume() << ". Known volume reached, terminate exploration.");
        return NO_FRONTIER;
      }
    }

    ed_->views_.clear();
    ed_->global_tour_.clear();

    std::cout << "start pos: " << pos.transpose() << "\nstart vel: " << vel.transpose()
              << "\nstart acc: " << acc.transpose() << std::endl;

    frontier_finder_->setCurrPos(pos, yaw[0]);
    Vector3d next_pos;
    double next_yaw;
    ROS_ERROR("[BC State] %s", frontier_finder_->behavior_state_str_[frontier_finder_->behavior_state_].c_str());

    is_planning_ = false;

    switch (frontier_finder_->behavior_state_)
    {
    case IN_CORRIDOR:
      // In corridor, search frontier, label ftr as corridor;
      // find nearest door->transit to NAVIGATE_TO_DOOR
      // If no nearest door, select best frontier and go

      // Search frontiers and group them into clusters
      frontier_finder_->searchFrontiersBC();

      // Doors should be detected only when we are in corridor or navigate to door state.
      // Frontiers should be saved either seperately or check label before doing door detection
      if (ep_->detect_semantics_) frontier_finder_->detectDoors();
      // Recheck existing door
      if (ep_->detect_semantics_) frontier_finder_->removeFPDoor();

      // Compute prediction on clusters.
      frontier_finder_->predOnFrontiers();
      frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitBC();
      frontier_finder_->recheckViewpoints();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      frontier_finder_->getFrontierPred(ed_->frontiers_occ_pred_);
      if (ep_->detect_semantics_) frontier_finder_->getDoors(ed_->door_pos_, ed_->door_yaws_);
      if (ed_->frontiers_.empty())
      {
        ROS_WARN("No coverable frontier.");
        return NO_FRONTIER;
      }

      // If has doors, navigate to the door instead of ftrs in corridor
      if (ep_->detect_semantics_ && frontier_finder_->setNextDoor(plan_fail_cnt_))
      {
        // set nearest door as goal -> move there to confirm door

        // this yaw should be fixed
        frontier_finder_->getDoorConfirmPos(next_pos, next_yaw);
        ROS_ERROR("Get door confirm POS: %f, %f, %f", next_pos(0), next_pos(1), next_pos(2));
        frontier_finder_->transitBCState(NAV_TO_DOOR, "BCSM");
        // is_Planning_
      }
      else
      {
        // No doors has found
        // Best utility frontier

        frontier_finder_->getTopViewpointsBC(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_, ed_->topo_ids_);
        // For viewpoint visualization
        for (unsigned int i = 0; i < ed_->points_.size(); ++i)
          ed_->views_.push_back(
              ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
        // FIXME: (YZ) why here 2.0 * yaw
        double view_time = (ros::Time::now() - t1).toSec();
        ROS_WARN(
            "Frontier: %lu, t: %lf, viewpoint: %lu, t: %lf", ed_->frontiers_.size(), frontier_time,
            ed_->points_.size(), view_time);

        t1 = ros::Time::now();
        int max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);

        double util_time = (ros::Time::now() - t1).toSec();
        ROS_WARN(
            "Utility: %f, Info Gain: %d, Cost: %f, t: %lf", ed_->utilities_[max_util_id], ed_->info_gains_[max_util_id],
            ed_->costs_[max_util_id], util_time);

        next_pos = ed_->points_[max_util_id];
        next_yaw = ed_->yaws_[max_util_id];
        // is_Planning_
      }
      is_planning_ = true;
      break;

    case NAV_TO_DOOR:
      // Check if reach confirm door pos
      // if reached, confirm
      // If confirmed, cross, transit to IN_ROOM
      // search frontier as well, label ftr as corridor;

      // Search frontiers and group them into clusters
      frontier_finder_->searchFrontiersBC();

      // Doors should be detected only when we are in corridor or navigate to door state.
      // Frontiers should be saved either seperately or check label before doing door detection
      if (ep_->detect_semantics_) frontier_finder_->detectDoors();
      // Recheck existing door
      if (ep_->detect_semantics_) frontier_finder_->removeFPDoor();

      // Compute prediction on clusters.
      frontier_finder_->predOnFrontiers();
      frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitBC();
      frontier_finder_->recheckViewpoints();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      frontier_finder_->getFrontierPred(ed_->frontiers_occ_pred_);
      if (ep_->detect_semantics_) frontier_finder_->getDoors(ed_->door_pos_, ed_->door_yaws_);

      // if reached confirm door position, need to confirm if it's a door
      if (ep_->detect_semantics_ && frontier_finder_->reachDoorConfirmPos())
      {
        ROS_ERROR("Reach confirm door pos");
        bool is_door = frontier_finder_->confirmDoor();
        ROS_ERROR("Confirm door: %d", is_door);
        if (is_door && frontier_finder_->getCrossDoorGoal(next_pos, next_yaw))
        {
          ROS_ERROR("Confirm door, Enter room!!!!");
          // Confirmed door, cross door now. Transit to room state
          // save the door crossed in frontier finder, later we need to go out
          // set door visited
          frontier_finder_->transitBCState(IN_ROOM, "BCSM");
        }
        else
        {
          ROS_ERROR("Confirm door false! Remove door");
          // Not door, remove it if still exist in door list
          frontier_finder_->removeDoorIfExist();
          // Re-find nearest door, replan;
          if (frontier_finder_->setNextDoor(plan_fail_cnt_))
          {
            ROS_ERROR("Confirm door false! find next door");
            frontier_finder_->getDoorConfirmPos(next_pos, next_yaw);
            // is_Planning_
          }
          else
          {
            ROS_ERROR("Confirm door false! no doors, go back to IN_CORRIDOR");
            // If no doors globally, transit to IN_CORRIDOR, find next best frontier
            frontier_finder_->transitBCState(IN_CORRIDOR, "BCSM");
            // No doors has found select best utility frontier
  
            frontier_finder_->getTopViewpointsBC(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_, ed_->topo_ids_);
            // For viewpoint visualization
            for (unsigned int i = 0; i < ed_->points_.size(); ++i)
              ed_->views_.push_back(
                  ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
            double view_time = (ros::Time::now() - t1).toSec();
            ROS_WARN(
                "Frontier: %lu, t: %lf, viewpoint: %lu, t: %lf", ed_->frontiers_.size(), frontier_time,
                ed_->points_.size(), view_time);
            t1 = ros::Time::now();
            int max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
            // If plan fails for multiple times, find another viewpoitns:
            if (plan_fail_cnt_ >= 3) {
              ROS_ERROR("Plan fails >= 3");
              ed_->points_.erase(ed_->points_.begin() + max_util_id);
              ed_->yaws_.erase(ed_->yaws_.begin() + max_util_id);
              ed_->info_gains_.erase(ed_->info_gains_.begin() + max_util_id);
              ed_->costs_.erase(ed_->costs_.begin() + max_util_id);
              ed_->utilities_.erase(ed_->utilities_.begin() + max_util_id);
              if (ed_->points_.size() == 0) return FAIL;
              max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
              plan_fail_cnt_ = 0;
            }

            double util_time = (ros::Time::now() - t1).toSec();
            ROS_WARN(
                "Utility: %f, Info Gain: %d, Cost: %f, t: %lf", ed_->utilities_[max_util_id], ed_->info_gains_[max_util_id],
                ed_->costs_[max_util_id], util_time);

            next_pos = ed_->points_[max_util_id];
            next_yaw = ed_->yaws_[max_util_id];
          }
        }
      }
      else
      {
        ROS_ERROR("Not Reach confirm door pos");

        // not reach the door, replan (re-find nearest door etc);
        if (ep_->detect_semantics_ && frontier_finder_->setNextDoor(plan_fail_cnt_))
        {
          frontier_finder_->getDoorConfirmPos(next_pos, next_yaw);
        }
        else
        {
          ROS_ERROR("BCSM: no door globally, transit to IN_CORRIDOR");
          // If no doors globally, transit to IN_CORRIDOR, find next best frontier
          frontier_finder_->transitBCState(IN_CORRIDOR, "BCSM");
          // No doors has found select best utility frontier

          frontier_finder_->getTopViewpointsBC(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_, ed_->topo_ids_);
          // For viewpoint visualization
          for (unsigned int i = 0; i < ed_->points_.size(); ++i)
            ed_->views_.push_back(
                ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
          double view_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Frontier: %lu, t: %lf, viewpoint: %lu, t: %lf", ed_->frontiers_.size(), frontier_time,
              ed_->points_.size(), view_time);
          t1 = ros::Time::now();
          int max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
          // If plan fails for multiple times, find another viewpoitns:
          if (plan_fail_cnt_ > 3) {
            ROS_ERROR("Plan fails > 3");
            ed_->points_.erase(ed_->points_.begin() + max_util_id);
            ed_->yaws_.erase(ed_->yaws_.begin() + max_util_id);
            ed_->info_gains_.erase(ed_->info_gains_.begin() + max_util_id);
            ed_->costs_.erase(ed_->costs_.begin() + max_util_id);
            ed_->utilities_.erase(ed_->utilities_.begin() + max_util_id);
            if (ed_->points_.size() == 0) return FAIL;
            max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
            plan_fail_cnt_ = 0;
          }
          double util_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Utility: %f, Info Gain: %d, Cost: %f, t: %lf", ed_->utilities_[max_util_id], ed_->info_gains_[max_util_id],
              ed_->costs_[max_util_id], util_time);

          next_pos = ed_->points_[max_util_id];
          next_yaw = ed_->yaws_[max_util_id];
        }
      }
      is_planning_ = true;
      break;

    case IN_ROOM:
      // In room, search frontier, label ftr as room;
      // Check if cross door pos reached. if not, directly replan
      // Select next best frontier from "room" and go
      // No door detection here!!!

      // Search frontiers and group them into clusters
      frontier_finder_->searchFrontiersBC();
      // Compute prediction on clusters.
      frontier_finder_->predOnFrontiers();
      frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitBC();
      frontier_finder_->recheckViewpoints();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      frontier_finder_->getFrontierPred(ed_->frontiers_occ_pred_);
      if (ep_->detect_semantics_) frontier_finder_->getDoors(ed_->door_pos_, ed_->door_yaws_);

      if (!frontier_finder_->room_visited_)
      {
        if (frontier_finder_->reachDoorCrossPos())
        {
          // label room as visited
          ROS_ERROR("Label room as visited!!!!!!!!!!!");
          frontier_finder_->labelRoomVisited();
        }
        else
        {
          // replan to door cross pos
          if (frontier_finder_->getCrossDoorGoal(next_pos, next_yaw)) {
            ROS_ERROR("Get cross door goal!!!!!!!!!!!");
            if (plan_fail_cnt_ >=3) {
              // given door cross pos, we couldnt find a path, abandon this door,
              // transit back to in_corridor state
              frontier_finder_->transitBCState(IN_CORRIDOR, "BCSM");
              plan_fail_cnt_ = 0;
            } else {
              is_planning_ = true;
              break;
            }
          } else {
            ROS_ERROR("Door cross pos invalid.");
            // transit back to in_corridor state
            frontier_finder_->transitBCState(IN_CORRIDOR, "BCSM");
          }

        }
      }

      ROS_ERROR("Label Room ftr");
      // Go through all ftrs, check and make sure they are in room
      frontier_finder_->labelRoomFtr();
      // If has ftr with type room, select and plan to there
      if (frontier_finder_->hasRoomFtr())
      {
        ROS_ERROR("Has Room ftr, select and go");
        ROS_ERROR("Has Room ftr, select and go");

        // Best utility frontier

        frontier_finder_->getTopViewpointsBC(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_, ed_->topo_ids_);
        // No good candidates inside room. Exit now!
        if (ed_->points_.size() == 0)
        {
          ROS_ERROR("No good viewpoint for ftr in room, exit");
          // If has no ftr with type room, we need to exit the room. Compute door exit pos
          // Transit to EXIT_DOOR
          frontier_finder_->getExitDoorGoal(next_pos, next_yaw);
          frontier_finder_->transitBCState(EXIT_ROOM, "BCSM");
          // is_Planning_
          break;
        }
        else
        {
          // For viewpoint visualization
          ROS_ERROR("Number of ROOM FTRS %d", ed_->points_.size());
          for (unsigned int i = 0; i < ed_->points_.size(); ++i)
            ed_->views_.push_back(
                ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
          double view_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Frontier: %lu, t: %lf, viewpoint: %lu, t: %lf", ed_->frontiers_.size(), frontier_time,
              ed_->points_.size(), view_time);

          t1 = ros::Time::now();
          int max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
          // If plan fails for multiple times, find another viewpoitns:
          if (plan_fail_cnt_ > 3) {
            ROS_ERROR("Plan fails > 3");
            ed_->points_.erase(ed_->points_.begin() + max_util_id);
            ed_->yaws_.erase(ed_->yaws_.begin() + max_util_id);
            ed_->info_gains_.erase(ed_->info_gains_.begin() + max_util_id);
            ed_->costs_.erase(ed_->costs_.begin() + max_util_id);
            ed_->utilities_.erase(ed_->utilities_.begin() + max_util_id);
            if (ed_->points_.size() == 0) return FAIL;
            max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
            plan_fail_cnt_ = 0;
          }

          double util_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Utility: %f, Info Gain: %d, Cost: %f, t: %lf", ed_->utilities_[max_util_id], ed_->info_gains_[max_util_id],
              ed_->costs_[max_util_id], util_time);

          next_pos = ed_->points_[max_util_id];
          next_yaw = ed_->yaws_[max_util_id];
          break;
        }
      }
      else
      {
        ROS_ERROR("No ftr in room, exit");
        // If has no ftr with type room, we need to exit the room. Compute door exit pos
        // Transit to EXIT_DOOR
        frontier_finder_->getExitDoorGoal(next_pos, next_yaw);
        frontier_finder_->transitBCState(EXIT_ROOM, "BCSM");
        break;
      }
      is_planning_ = true;
      break;

    case EXIT_ROOM:
      // EXIT ROOM, first check if reach exit pos, if yes, transit to IN_CORRIDOR, select next best FTR
      // If not reach exit pos, replan to exit pos

      // Search frontiers and group them into clusters
      frontier_finder_->searchFrontiersBC();

      // Doors should be detected only when we are in corridor or navigate to door state.
      // Frontiers should be saved either seperately or check label before doing door detection
      // frontier_finder_->detectDoors();
      // Recheck existing door
      // frontier_finder_->removeFPDoor();

      // Compute prediction on clusters.
      frontier_finder_->predOnFrontiers();
      frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitBC();
      frontier_finder_->recheckViewpoints();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      frontier_finder_->getFrontierPred(ed_->frontiers_occ_pred_);
      if (ep_->detect_semantics_) frontier_finder_->getDoors(ed_->door_pos_, ed_->door_yaws_);

      if (frontier_finder_->reachDoorExitPos())
      {
        frontier_finder_->transitBCState(IN_CORRIDOR, "BCSM");
        // Should check if there are more doors to go.
        if (frontier_finder_->setNextDoor(plan_fail_cnt_))
        {
          frontier_finder_->getDoorConfirmPos(next_pos, next_yaw);
        }
        else
        {

          frontier_finder_->getTopViewpointsBC(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_, ed_->topo_ids_);
          // For viewpoint visualization
          for (unsigned int i = 0; i < ed_->points_.size(); ++i)
            ed_->views_.push_back(
                ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
          double view_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Frontier: %lu, t: %lf, viewpoint: %lu, t: %lf", ed_->frontiers_.size(), frontier_time,
              ed_->points_.size(), view_time);
          t1 = ros::Time::now();
          int max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
          // If plan fails for multiple times, find another viewpoitns:
          if (plan_fail_cnt_ > 3) {
            ROS_ERROR("Plan fails > 3");
            ed_->points_.erase(ed_->points_.begin() + max_util_id);
            ed_->yaws_.erase(ed_->yaws_.begin() + max_util_id);
            ed_->info_gains_.erase(ed_->info_gains_.begin() + max_util_id);
            ed_->costs_.erase(ed_->costs_.begin() + max_util_id);
            ed_->utilities_.erase(ed_->utilities_.begin() + max_util_id);
            if (ed_->points_.size() == 0) return FAIL;
            max_util_id = computeFrontierUtilities(pos, vel, yaw, ed_->points_, ed_->yaws_, ed_->info_gains_, ed_->costs_, ed_->utilities_);
            plan_fail_cnt_ = 0;
          }
          double util_time = (ros::Time::now() - t1).toSec();
          ROS_WARN(
              "Utility: %f, Info Gain: %d, Cost: %f, t: %lf", ed_->utilities_[max_util_id], ed_->info_gains_[max_util_id],
              ed_->costs_[max_util_id], util_time);

          next_pos = ed_->points_[max_util_id];
          next_yaw = ed_->yaws_[max_util_id];
        }
      }
      else
      {
        // Not yet exit door, replan to exit door.
        // Select next best ftr
        frontier_finder_->getExitDoorGoal(next_pos, next_yaw);
      }
      is_planning_ = true;
      break;
    }

    if (is_planning_)
    {
      std::cout << "[=== 0. Setting the replan next goal as === ]: " << next_pos.transpose() << " " << next_yaw << std::endl;
      replan_next_pos_ = next_pos;
      replan_next_yaw_ = next_yaw;
      int plan_status = planBC(next_pos, next_yaw, pos, vel, acc, yaw);
      return plan_status;
    }
  }

  int bgExplorationManager::planBC(const Vector3d &next_pos,
                                   const double &next_yaw,
                                   const Vector3d &pos,
                                   const Vector3d &vel,
                                   const Vector3d &acc,
                                   const Vector3d &yaw)
  {
    std::cout << "[=== 1. Init planning status as === ]:\nstart pos: " << pos.transpose()
              << "\nstart vel: " << vel.transpose()
              << "\nstart acc: " << acc.transpose()
              << "\nstart yaw: " << yaw.transpose()
              << "\ngoal  next_pos: " << next_pos.transpose()
              << "\ngoal  next_yaw: " << next_yaw << std::endl;

    // Visualization for Yuwei
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id += 1;
    mk.ns = "next_goal";
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.color.r = 0.8 * double(mk.id) / 1000;
    mk.color.g = 0;
    mk.color.b = 0.8;
    mk.color.a = 1;
    mk.scale.x = 0.8;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    // Roll pitch and yaw in Radians
    double tmp_roll = 0, tmp_pitch = 0, tmp_yaw = next_yaw;
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(tmp_roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(tmp_pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(tmp_yaw, Eigen::Vector3f::UnitZ());
    mk.pose.position.x = next_pos(0);
    mk.pose.position.y = next_pos(1);
    mk.pose.position.z = next_pos(2);
    mk.pose.orientation.w = q.w();
    mk.pose.orientation.x = q.x();
    mk.pose.orientation.y = q.y();
    mk.pose.orientation.z = q.z();
    // mk.action = visualization_msgs::Marker::ADD;
    next_goal_pub_.publish(mk);

    if (plan_fail_cnt_ >= 2)
    {
      // plan_fail_cnt_ = 0.0;
      ROS_WARN("[bgExplorationManager::planBC] Fail too much timel, rotate first!" );
      return ROTATE;
    }

    // Plan trajectory (position and yaw) to the next viewpoint
    ros::Time t1 = ros::Time::now();

    Eigen::MatrixXd start_state(3, 3); // include the start and end state
    start_state << pos, vel, acc;

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = next_yaw - yaw[0];
    while (diff < -M_PI)
      diff += 2 * M_PI;
    while (diff >= M_PI)
      diff -= 2 * M_PI;

    diff = abs(diff);
    diff = min(diff, 2 * M_PI - diff);
    double time_lb = diff / ViewNode::yd_; // NOTE: yd_=60 degree in config

    ed_->next_goal_ = next_pos;
    bool success1 = false, success2 = true;
    // Generate trajectory of x,y,z
    double goal_dist = (next_pos - pos).norm();

    std::cout << "[=== 2. Goal evaluation   === ] \ngoal_dist is : " << goal_dist << ", the yaw diff is : " << diff  << std::endl;

    if (goal_dist < radius_noplan_ && diff < 0.2)
    {
      plan_fail_cnt_++;
      ROS_WARN("[bgExplorationManager] Goal is too close and no need for rotation, end ..." );
      return FAIL;
    }
    else if (goal_dist <= radius_noplan_)
    {
      /******************************************************************/
      /**********************The goal is very close**********************/
      /******************************************************************/
      ROS_WARN("[bgExplorationManager] Goal is too close but need for rotation ..." );
      auto  plan_t1 = ros::Time::now();
      success1 = planner_manager_->stay(start_state, time_lb);
      auto  plan_t2 = ros::Time::now();
      ROS_WARN("Planning time: %lf", (plan_t2-plan_t1).toSec());

      if (success1)
      {
        success2 = planner_manager_->planYawNoExplore(yaw, next_yaw, false, 0.5);
        if(!success2)
        {
          success2 = planner_manager_->planYawNoExplore(yaw, next_yaw, true, 0.0);
        }

        ROS_WARN("Yaw Planning time: %lf", (ros::Time::now() - plan_t2).toSec());

      }else
      {
        ROS_WARN("======================== Planning fails... no need for yaw planning ");
      }

    }
    else
    {
      /******************************************************************/
      /*The goal is valid and should heading to the trajectory first */
      /******************************************************************/
      // call a* first to decide whether we should head the trajectory
      ROS_WARN("[bgExplorationManager] The goal is valid and should heading to the trajectory first" );

      if (goal_dist > radius_close_)
      { 
        ROS_WARN("Call A star planning first to get a near close goal ...");
        auto  plan_t0 = ros::Time::now();

        planner_manager_->path_finder_->reset();
        if (planner_manager_->path_finder_->search(pos, next_pos) == Astar::REACH_END)
        {
          ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
          shortenPath(ed_->path_next_goal_);
          // Next viewpoint is far away
          double len2 = 0.0;
          vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
          for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far_; ++i)
          {
            auto cur_pt = ed_->path_next_goal_[i];
            len2 += (cur_pt - truncated_path.back()).norm();
            truncated_path.push_back(cur_pt);
          }
          ed_->next_goal_ = truncated_path.back();

          ROS_WARN("A star Planning time: %lf", (ros::Time::now()-plan_t0).toSec());
        }

       
      }
    
      //check rotation diff of two points
      auto dir = ed_->next_goal_.head(2) - pos.head(2);
      double h_yaw = atan2(dir(1), dir(0));

      auto  plan_t1 = ros::Time::now();
      success1 = planner_manager_->planExploreTraj(ed_->next_goal_, start_state, h_yaw);
      ROS_WARN("Planning time: %lf", (ros::Time::now()-plan_t1).toSec());

      if (success1)
      {
        auto  plan_t2 = ros::Time::now();
        // compute h_diff
        double h_diff = h_yaw - yaw[0];
        while (h_diff < -M_PI)
          h_diff += 2 * M_PI;
        while (h_diff >= M_PI)
          h_diff -= 2 * M_PI;
        h_diff = abs(h_diff);
        h_diff = min(h_diff, 2 * M_PI - h_diff);
        time_lb = h_diff / ViewNode::yd_;

        if (goal_dist <= radius_close_)
        {
          /******************************************************************/
          /***The goal is approach and we should rotate to target yaw ... ***/
          /******************************************************************/
          ROS_WARN("[Yaw Planning] The goal is approach and we should rotate to target yaw ..." );
          success2 = planner_manager_->planYawNoExplore(yaw, next_yaw, false, 0.0);
        }
        else if (goal_dist > radius_close_ && goal_dist <= radius_far_)
        {
          /***The goal is within range and we can do exploration ... ***/
          ROS_WARN("[Yaw Planning] The goal is within range and we can do exploration ..." );
          success2 = planner_manager_->planYawExplore(yaw, next_yaw);
        }
        else
        {
          /***The goal is far away or need look ahead first... ***/
          ROS_WARN("[Yaw Planning] The goal is far away or need look ahead first..." );
          if (h_diff > 0.5 * v_fov_)
          {
            success2 = planner_manager_->planYawNoExplore(yaw, h_yaw, false, 0.0);
          }else{
            success2 = planner_manager_->planYawNoExplore(yaw, h_yaw, true, 0.0);
          }
        }

        ROS_WARN("Yaw Planning time: %lf", (ros::Time::now()-plan_t2).toSec());

      }else
      {
        ROS_WARN("======================== Planning fails... no need for yaw planning ");
      }

    }

    auto total = (ros::Time::now() - t1).toSec();
    ROS_WARN("Total time is : %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    if (success1 && success2)
    {
      plan_fail_cnt_ = 0;
      ROS_WARN("======================== Planning success!!!!!!!!!!!!!!!!!!");
      return SUCCEED;
    }
    plan_fail_cnt_++;
    return FAIL;
  }

  // Return index of viewpoint which has highest utility
  int bgExplorationManager::computeFrontierUtilities(
      const Vector3d &cur_pos, const Vector3d &vel, const Vector3d &cur_yaw,
      const vector<Eigen::Vector3d> &points, const vector<double> &yaws,
      const vector<int> &info_gains, vector<double> &costs, vector<double> &utilities)
  {
    costs.clear();
    utilities.clear();
    ROS_WARN("vp size: %ld", points.size());
    double max_util = -1.0;
    double max_util_id = -1;
    for (unsigned int i = 0; i < points.size(); i++)
    {
      vector<Vector3d> tmp_path;
      // double tmp_cost = ViewNode::computeCost(cur_pos, points[i], cur_yaw[0], yaws[i], vel, cur_yaw[1], tmp_path);
      double tmp_cost = ViewNode::computePathTimeCost(cur_pos, points[i], cur_yaw[0], yaws[i], vel, cur_yaw[1], tmp_path);
      double tmp_util = info_gains[i] / tmp_cost;
      costs.push_back(tmp_cost);
      utilities.push_back(tmp_util);
      if (tmp_util > max_util)
      {
        max_util = tmp_util;
        max_util_id = i;
      }
    }
    ROS_WARN("Finish compute utility");
    return max_util_id;
  }

  void bgExplorationManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (unsigned int i = 1; i < path.size() - 1; ++i)
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

} // namespace bg_planner
