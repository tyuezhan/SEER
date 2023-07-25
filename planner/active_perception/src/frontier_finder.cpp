#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>


#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace bg_planner {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/exp_3d", exp_3d_, false);
  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/room_min_candidate_dist", room_min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/min_candidate_obs_clearance", min_candidate_obs_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_info_gain", min_info_gain_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/pred_size_x", pred_size_x_, 80);
  nh.param("frontier/pred_size_y", pred_size_y_, 80);
  nh.param("frontier/pred_size_z", pred_size_z_, 40);
  nh.param("frontier/h_fov", h_fov_, -1.0);
  nh.param("frontier/v_fov", v_fov_, -1.0);
  nh.param("frontier/min_ray_length", min_ray_length_, -1.0);
  nh.param("frontier/max_ray_length", max_ray_length_, -1.0);
  nh.param("frontier/door_confirm_thres", door_confirm_thres_, 1.0);
  nh.param("frontier/door_cross_thres", door_cross_thres_, 1.0);
  nh.param("frontier/door_exit_thres", door_exit_thres_, 1.0);
  nh.param("frontier/door_confirm_dis", door_confirm_d_, 1.0);
  nh.param("frontier/door_cross_dis", door_cross_d_, 1.0);
  nh.param("frontier/door_exit_dis", door_exit_d_, 1.0);
  nh.param("frontier/h_min_line_length", h_min_line_length_, 1.0);
  nh.param("frontier/h_max_line_gap", h_max_line_gap_, 1.0);
  nh.param("frontier/v_min_line_length", v_min_line_length_, 1.0);
  nh.param("frontier/v_max_line_gap", v_max_line_gap_, 1.0);
  nh.param("frontier/h_min_door", h_min_door_, 100);
  nh.param("frontier/h_max_door", h_max_door_, 100);
  nh.param("frontier/v_min_door", v_min_door_, 100);
  nh.param("frontier/v_max_door", v_max_door_, 100);
  // minimum number of votes (intersections in Hough grid cell
  nh.param("frontier/min_vote", min_vote_, 6);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  percep_utils_.reset(new PerceptionUtils(nh));

  global_ftr_buffer_id_ = 0;
  global_topo_node_id_ = 0;
  current_topo_node_id_ = 0;
  ftr_counter_ = 0;
  h_ftr_counter_ = 0;
  global_door_id_ = 0;
  pred_client_ = nh.serviceClient<occ_predictor_msgs::PredictPCL>("/frontier/occ_map/pred");
  behavior_state_ = BEHAVIOR_STATE::IN_CORRIDOR;
  behavior_state_str_ = {"IN_CORRIDOR", "NAV_TO_DOOR", "IN_ROOM", "EXIT_ROOM"};
  door_goal_set_ = false;
  has_room_ftr_ = false;
  room_visited_ = false;

  ROS_INFO("check enum: %d, %d, %d", edt_env_->sdf_map_->UNKNOWN, edt_env_->sdf_map_->FREE, edt_env_->sdf_map_->OCCUPIED);

}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::searchFrontiersBC() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;

  // Note: here, the updated box will either be update_min, update_max if it is updated,
  // Otherwise, it will be camera_pos
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed frontiers in updated map 
  // Remove changed frontiers' buffers as well
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    all_local_occ_buffer_.erase(iter->buf_id_);
    all_pred_buffer_.erase(iter->buf_id_);
    all_global_pos_buffer_.erase(iter->buf_id_);
    all_local_box_id_min_.erase(iter->buf_id_);
    all_local_box_id_max_.erase(iter->buf_id_);

    iter = frontiers.erase(iter);
  };

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }

  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }
  splitLargeFrontiers(tmp_frontiers_);

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
  
  // Prediction part
  makeFrontierBuffers(tmp_frontiers_);
  
}



void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first) {
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.4) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    frontier.buf_id_ = -1;
    frontier.has_pred_ = false;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;

  // any cell's distance to center greater than cluster size
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

// Make local occupancy buffer and prediction buffer for 3D occupancy prediction
void FrontierFinder::makeFrontierBuffers(list<Frontier>& frontiers) {
  auto t1 = ros::Time::now();
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) { 
    std::vector<double> pred_buf;
    std::vector<int> local_buf;
    std::vector<Vector3d> global_pos_buffer;
    Eigen::Vector3i local_box_min, local_box_max;
    local_box_min << -1, -1, -1;
    local_box_max << -1, -1, -1;
    // std::vector<int> local_idx_buffer_;
    int pred_buf_size = pred_size_x_ * pred_size_y_ * pred_size_z_;
    local_buf.resize(pred_buf_size);
    pred_buf.resize(pred_buf_size);
    global_pos_buffer.resize(pred_buf_size);
    // local_idx_buffer_.reserve(pred_buf_size);
    fill(local_buf.begin(), local_buf.end(), edt_env_->sdf_map_->getMapDefault());
    fill(pred_buf.begin(), pred_buf.end(), edt_env_->sdf_map_->mp_->clamp_min_log_ - edt_env_->sdf_map_->mp_->unknown_flag_);
    fill(global_pos_buffer.begin(), global_pos_buffer.end(), Vector3d {0,0,0});
    global_ftr_buffer_id_++;

    all_local_occ_buffer_.emplace(global_ftr_buffer_id_, local_buf);
    all_pred_buffer_.emplace(global_ftr_buffer_id_, pred_buf);
    all_global_pos_buffer_.emplace(global_ftr_buffer_id_, global_pos_buffer);
    all_local_box_id_min_.emplace(global_ftr_buffer_id_, local_box_min);
    all_local_box_id_max_.emplace(global_ftr_buffer_id_, local_box_max);
    // all_local_idx_buffer_.emplace(global_ftr_buffer_id_, local_idx_buffer_);
    it->buf_id_ = global_ftr_buffer_id_;
  }
  ROS_WARN_THROTTLE(5.0, "makeFrontierBuffers t: %lf", (ros::Time::now() - t1).toSec());
}

// Make topological nodes from a frontier node for building topological map
void FrontierFinder::makeTopoNode(Frontier& ftr) {
  auto t1 = ros::Time::now();
  TopoNode tmp_node;
  global_topo_node_id_ ++;
  tmp_node.topo_id_ = global_topo_node_id_;
  ftr.topo_id_ = global_topo_node_id_;
  tmp_node.type_ = -1;
  tmp_node.parent_id_ = current_topo_node_id_;
  // When create topo node, set pos = cluster center, update later
  tmp_node.pos_ = ftr.average_;
  topo_nodes_.emplace(global_topo_node_id_, tmp_node);
  
  ROS_WARN_THROTTLE(5.0, "makeTopoNode t: %lf", (ros::Time::now() - t1).toSec());
}

// For all new frontiers, compute 
void FrontierFinder::predOnFrontiers() {
  auto t1 = ros::Time::now();
  int map_voxel_num_yz = edt_env_->sdf_map_->mp_->map_voxel_num_(1) * edt_env_->sdf_map_->mp_->map_voxel_num_(2);
  
  Vector3d cluster_center, pred_range, local_pred_range_min, local_pred_range_max;
  double map_resolution = edt_env_->sdf_map_->getResolution();
  pred_range(0) = pred_size_x_ * map_resolution;
  pred_range(1) = pred_size_y_ * map_resolution;
  pred_range(2) = pred_size_z_ * map_resolution;
  Eigen::Vector3i min_id, max_id;
  // For each frontier cluster, do prediction
  for (auto it = tmp_frontiers_.begin(); it != tmp_frontiers_.end(); ++it) { 
    auto t_now = ros::Time::now();
    double t_used = (t_now-t1).toSec();
    if (t_used > 1.0) {
      ROS_WARN("Inference timeout");
      break;
    }
    occ_predictor_msgs::PredictPCL pred_srv;
    // First, extract local buffer
    cluster_center = it->average_;
    local_pred_range_min = cluster_center - pred_range/2;
    local_pred_range_max = cluster_center + pred_range/2;
    local_pred_range_min(2) = 0;
    local_pred_range_max(2) = pred_range(2);
    Eigen::Vector3i box_min_id, box_max_id;
    edt_env_->sdf_map_->posToIndex(local_pred_range_min, box_min_id);
    edt_env_->sdf_map_->posToIndex(local_pred_range_max, box_max_id);
    
    Eigen::Vector3i box_size;
    box_size = box_max_id - box_min_id;
    // For mem safe operation
    if (box_size(0) != pred_size_x_ || box_size(1) != pred_size_y_ || box_size(2) != pred_size_z_) {
      box_size(0) = pred_size_x_;
      box_size(1) = pred_size_y_;
      box_size(2) = pred_size_z_;
      box_max_id(0) = box_min_id(0) + pred_size_x_;
      box_max_id(1) = box_min_id(1) + pred_size_y_;
      box_max_id(2) = box_min_id(2) + pred_size_z_;
    }
    // ROS_INFO("local buffer size, %d, %d, %d", box_size(0), box_size(1), box_size(2));
    
    // Save local box min and box max
    all_local_box_id_min_[it->buf_id_] = box_min_id;
    all_local_box_id_max_[it->buf_id_] = box_max_id;

    // Copy from occupancy buffer to local buffer.
    int local_idx;
    Eigen::Vector3i global_id;
    Eigen::Vector3d global_pos;
    bool in_map;
    for (int id_x = box_min_id(0); id_x < box_max_id(0); id_x++) {
      for (int id_y = box_min_id(1); id_y < box_max_id(1); id_y++) {
        for (int id_z = box_min_id(2); id_z < box_max_id(2); id_z++) {
          local_idx = (id_x - box_min_id(0)) * box_size(1) * box_size(2) +
                      (id_y - box_min_id(1)) * box_size(2) +
                      (id_z - box_min_id(2));
          global_id(0) = id_x;
          global_id(1) = id_y;
          global_id(2) = id_z;
          in_map = edt_env_->sdf_map_->isInMap(global_id);
          // Save occ value (UNKNOWN=-1, FREE=0, OCC=1)
          if (in_map) all_local_occ_buffer_[it->buf_id_][local_idx] = edt_env_->sdf_map_->getOccupancy(global_id);
          else all_local_occ_buffer_[it->buf_id_][local_idx] = edt_env_->sdf_map_->getMapDefault();
          
          // Save global pos
          edt_env_->sdf_map_->indexToPos(global_id, global_pos);
          all_global_pos_buffer_[it->buf_id_][local_idx] = global_pos;
        }
      } 
    }
    // Make prediction service call to net_node
    pred_srv.request.dim_x = pred_size_x_;
    pred_srv.request.dim_y = pred_size_y_;
    pred_srv.request.dim_z = pred_size_z_;
    for (int i = 0; i < int(all_local_occ_buffer_[it->buf_id_].size()); i++) {
      pred_srv.request.input.push_back(all_local_occ_buffer_[it->buf_id_][i]);
    }

    // Get back results and save to pred_buffer
    if (!ros::service::waitForService("/frontier/occ_map/pred", ros::Duration(5))) ROS_ERROR("[PredOnFrontiers] Service not available");
    if (pred_client_.call(pred_srv)) {
      it->has_pred_ = true;
      // Copy pred to local pred buffer
      for (int j = 0; j < pred_size_x_ * pred_size_y_ * pred_size_z_; j++) {
        if (pred_srv.response.pred[j] > edt_env_->sdf_map_->mp_->pred_occupancy_log_) {
          // Occupied
          all_pred_buffer_[it->buf_id_][j] = edt_env_->sdf_map_->mp_->clamp_max_log_;
        } 
        // Unknown
        else if (pred_srv.response.pred[j] < edt_env_->sdf_map_->mp_->clamp_min_log_ - 1e-3) {
          all_pred_buffer_[it->buf_id_][j] = edt_env_->sdf_map_->mp_->clamp_min_log_ - edt_env_->sdf_map_->mp_->unknown_flag_;
        } 
        // Free
        else {
          all_pred_buffer_[it->buf_id_][j] = edt_env_->sdf_map_->mp_->clamp_min_log_;
        }
      }
    } else {
      ROS_ERROR("Failed to call service PredictPCL");
      it->has_pred_ = false;
    }
  }
  ROS_WARN_THROTTLE(5.0, "PredOnFrontiers t: %lf", (ros::Time::now() - t1).toSec());
}


bool FrontierFinder::isInBoxes(
    const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}


bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());
  ftr.has_pred_ = false;
  if (behavior_state_ == IN_CORRIDOR || behavior_state_ == NAV_TO_DOOR || behavior_state_ == EXIT_ROOM) {
    ftr.type_ = CORRIDOR;
  } else {
    ftr.type_ = ROOM;
  }

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}


void FrontierFinder::recheckViewpoints() {
  for (auto ftr = frontiers_.begin(); ftr != frontiers_.end();) {
    for (auto viewpoint = ftr->viewpoints_.begin(); viewpoint != ftr->viewpoints_.end();) {
      if (edt_env_->sdf_map_->getInflateOccupancy(viewpoint->pos_) == 1 || isNearUnknown(viewpoint->pos_) || isNearObstacle(viewpoint->pos_)) {
        viewpoint = ftr->viewpoints_.erase(viewpoint);
      } else {
        ++viewpoint;
      }
    }
    if (ftr->viewpoints_.empty()) {
      // remove ftr. push it to dormant
      dormant_frontiers_.push_back(*ftr);
      ftr = frontiers_.erase(ftr);
    }
    else {
      ++ftr;
    }
  }
}


void FrontierFinder::computeFrontiersToVisitBC() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // NOTE (YZ): sample viewpoint from tmp_frontiers, insert to frontiers_
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpointsIG(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      // has viewpoint but all viewpoints are too close. Discard this in room ftr. 
      if (behavior_state_ == IN_ROOM) {
        double no_view = true;
        for (auto iter = tmp_ftr.viewpoints_.begin(); iter != tmp_ftr.viewpoints_.end(); ++iter) {
          if ((iter->pos_ - cur_pos_).norm() > room_min_candidate_dist_) {
            no_view = false;
            break;
          } 
        }
        if (no_view) {
          dormant_frontiers_.push_back(tmp_ftr);
          ++new_dormant_num;
          continue;
        } 
      }
      ++new_num;
      // NOTE (YZ): iterator points to the first of the newly inserted element
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by predicted info gain, best view in front
      sort(
          inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.info_gain_ > v2.info_gain_; });
      // NOTE (YZ): make first new ftr point to newly inserted one (but only the first newly inserted)
      // Basically record the starting position of newly inserted one for future use
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      // Find no viewpoint, move cluster to dormant list
      // NOTE (YZ): tmp_ftr doesn't find good viewpoint
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}



void FrontierFinder::labelRoomFtr(const double& d) {
  ros::Time t1 = ros::Time::now();
  for (auto iter = frontiers_.begin(); iter != frontiers_.end(); ++iter) {
    if (iter->type_ == CORRIDOR) {
      continue;
    } else {
      double labeled = false;
      ROS_ERROR("Search path to ROOM FTR");
      vector<Vector3d> path;
      ViewNode::searchRawPath(cur_pos_, iter->viewpoints_.front().pos_, path);
      if (path.size() == 0) {
        iter->type_ = CORRIDOR;
        labeled = true;
        continue;
      }
      ROS_ERROR("path size: %d", path.size());
      for (auto node: path) {
        if (pow(node(0) - door_goal_pos_(0), 2) + pow(node(1) - door_goal_pos_(1), 2) < d*d) {
          // path to ftr cross door, indicate it's outside room
          iter->type_ = CORRIDOR;
          labeled = true;
          ROS_ERROR("Find CORRIDOR FTR labelled as Room");
          break;
        }
      }
      if (! labeled) {
        ROS_ERROR("Search path to ROOM FTR average!!!!!!");
        vector<Vector3d> path2;
        ViewNode::searchRawPath(cur_pos_, iter->average_, path2);
        if (path.size() == 0) {
          iter->type_ = CORRIDOR;
          continue;
        }
        ROS_ERROR("path2 size: %d", path.size());
        for (auto node2: path2) {
          if (pow(node2(0) - door_goal_pos_(0), 2) + pow(node2(1) - door_goal_pos_(1), 2) < d*d) {
            // path to ftr cross door, indicate it's outside room
            iter->type_ = CORRIDOR;
            ROS_ERROR("Find CORRIDOR FTR labelled as Room");
            break;
          }
        }
      }
    }
  }  
  ROS_WARN_THROTTLE(5.0, "Ftr semantic labeling t: %lf", (ros::Time::now() - t1).toSec());
}


bool FrontierFinder::hasRoomFtr() {
  for (auto iter = frontiers_.begin(); iter != frontiers_.end(); ++iter) {
    if (iter->type_ == ROOM) return true;
  }
  return false;
}


void FrontierFinder::getTopViewpointsBC(
  const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws,
  vector<Eigen::Vector3d>& averages, vector<int>& info_gains, vector<int>& topo_ids) {
  points.clear();
  yaws.clear();
  averages.clear();
  info_gains.clear();
  topo_ids.clear();
  // NOTE (YZ): for each of the frontier clusters, retrieve the viewpoint gives highest gain
  for (auto frontier : frontiers_) {
    if (behavior_state_ == IN_CORRIDOR || behavior_state_ == NAV_TO_DOOR || behavior_state_ == EXIT_ROOM) {
      if (frontier.type_ != CORRIDOR) continue;
    } else {
      if (frontier.type_ != ROOM) continue;
    }

    bool no_view = true;
    // If no view, pass
    if (frontier.viewpoints_.size() == 0) continue;

    for (auto view : frontier.viewpoints_) {
      ROS_ERROR("Has view !!! Info gain %d", view.info_gain_);
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      info_gains.push_back(view.info_gain_);
      topo_ids.push_back(frontier.topo_id_);
      no_view = false;
      break;
    }

  }
}


void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
}


void FrontierFinder::getFrontierObjs(list<Frontier> &cluster_objs) {
  cluster_objs.clear();
  for (auto frontier : frontiers_)
    cluster_objs.push_back(frontier);
}


// Save all predicted occupied cells' positions at each each frontier cluster
void FrontierFinder::getFrontierPred(vector<vector<Eigen::Vector3d>>& preds) {
  preds.clear();
  ROS_INFO("[getFrontierPred] clear preds.");
  for (auto frontier : frontiers_) {
    if (frontier.has_pred_ == true) {
      // ROS_INFO("[getFrontierPred] Frontier has prediction.");
      vector<Eigen::Vector3d> pred_occ_pos;
      for (int i = 0; i < pred_size_x_ * pred_size_y_ * pred_size_z_; i++) {
        if (all_pred_buffer_[frontier.buf_id_][i] == edt_env_->sdf_map_->mp_->clamp_max_log_) {
          // Note: cut cells below 1.5m
          if (all_global_pos_buffer_[frontier.buf_id_][i](2) < 0.2) continue;
          if (all_global_pos_buffer_[frontier.buf_id_][i](2) < 2.0)
          pred_occ_pos.push_back(all_global_pos_buffer_[frontier.buf_id_][i]);
        }
      }
      preds.push_back(pred_occ_pos);
    }
  }
}


void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}


void FrontierFinder::getDoors(vector<Vector3d>& pos, vector<double>& yaws) {
  pos.clear();
  yaws.clear();
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    pos.push_back(iter->pos_);
    yaws.push_back(iter->yaw_);
  }
}


void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}


void FrontierFinder::sampleViewpointsIG(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that give most prediction information gain
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr) {
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      if (!exp_3d_) frontier.average_(2) = 1.0;
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos) || isNearObstacle(sample_pos))
        continue;

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      vector<Viewpoint> tmp_vps;

      // Compute predicted information gain for sampled pos
      // Note: currently return at most 3 yaw at each pos. return only best if not needed
      // Note: currently return best at each pos
      if (frontier.has_pred_ == false) {
        ROS_ERROR("Trying to sample from frontier has no prediction");
      }
      computeInfoGain(sample_pos, avg_yaw, frontier.buf_id_, tmp_vps);
      for (auto vp : tmp_vps) {
        frontier.viewpoints_.push_back(vp);
      }
    }
  }
}  


bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos, double clearance) {
  int vox_num;
  if (clearance < 0) vox_num = floor(min_candidate_clearance_ / resolution_);
  else vox_num = floor(clearance / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}


bool FrontierFinder::isNearObstacle(const Eigen::Vector3d& pos, double clearance) {
  int vox_num;
  if (clearance < 0) vox_num = floor(min_candidate_obs_clearance_ / resolution_);
  else const int vox_num = floor(clearance / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -4; z <= 4; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::OCCUPIED) return true;
      }
  return false;
}


int FrontierFinder::computeYawInfoGain(const Eigen::Vector3d& pos, 
                                       const double& yaw) 
{
  double half_v_fov, min_ray_length, max_ray_length, half_h_fov; // should be get from camera config
  half_v_fov = v_fov_/2;
  half_h_fov = h_fov_/2;
  max_ray_length = max_ray_length_;
  min_ray_length = min_ray_length_;
  // yaw is average yaw from current position to frontier clusters 

  int gain = 0;
  for (auto &frontier: frontiers_)
  {
    int buf_id = frontier.buf_id_;
    for (double phi_h = -half_h_fov; phi_h <= half_h_fov; phi_h += half_h_fov / 3) {
      for (double phi_v = -half_v_fov; phi_v <= half_v_fov; phi_v += half_v_fov / 3) {
        // Find endpoint of a ray
        Vector3d end, start;
        end[0] = pos[0] + max_ray_length * cos(phi_v) * cos(yaw + phi_h);
        end[1] = pos[1] + max_ray_length * cos(phi_v) * sin(yaw + phi_h);
        end[2] = pos[2] + max_ray_length * sin(phi_v);
        start[0] = pos[0] + min_ray_length * cos(phi_v) * cos(yaw + phi_h);
        start[1] = pos[1] + min_ray_length * cos(phi_v) * sin(yaw + phi_h);
        start[2] = pos[2] + min_ray_length * sin(phi_v);
        // Do raycasting to check info gain
        Eigen::Vector3i idx;
        bool in_pred_box = false;
        int local_idx;
        bool flag_unknown = false;
        raycaster_->input(start, end);
        while (raycaster_->nextId(idx)) {
          // Idx is global idx in map, check occupancy first
          // If hit obstacle, break
          if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
            break;
          // Cell is unknown in the original map
          if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
            // New way:
            // First check if cell in local pred map
            in_pred_box = globalInPredBox(idx, buf_id);
            if (!in_pred_box) {
              // Not in pred map, count gain assuming unknown -> gain
              gain ++;
              continue;
            };
            globalToPredIdx(idx, local_idx, buf_id);
            // If pred is unknown
            if (all_pred_buffer_[buf_id][local_idx] < edt_env_->sdf_map_->mp_->clamp_min_log_ - 1e-3) {
              gain ++;
            }
            // If pred is occupied
            else if (all_pred_buffer_[buf_id][local_idx] > edt_env_->sdf_map_->mp_->min_occupancy_log_) {
              gain ++;
              break;
            }
            // If pred is free
            else gain ++;
          }
          // If cell is free, continue
        }
      }
    }

  }
  return gain;

}


// Given a sample pos, a frontier cluster yaw, buf_id, return viewpoints provides best info gain
void FrontierFinder::computeInfoGain(const Eigen::Vector3d& pos, const double& yaw, const int& buf_id, vector<Viewpoint>& vps) {
  double half_v_fov, min_ray_length, max_ray_length, half_h_fov; // should be get from camera config
  half_v_fov = v_fov_/2;
  half_h_fov = h_fov_/2;
  max_ray_length = max_ray_length_;
  min_ray_length = min_ray_length_;
  // yaw is average yaw from current position to frontier clusters 
  vector<int> slice_gains;

  // Evaluate info gain of different slices, horizontally 18 slices, 10 degree each
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice at different height (vertical angle)
    int gain = 0;
    int potential_gain = 0;
    for (double phi_v = -half_v_fov; phi_v <= half_v_fov; phi_v += half_v_fov / 3) {
      // Find endpoint of a ray
      Vector3d end, start;
      end[0] = pos[0] + max_ray_length * cos(phi_v) * cos(yaw + phi_h);
      end[1] = pos[1] + max_ray_length * cos(phi_v) * sin(yaw + phi_h);
      end[2] = pos[2] + max_ray_length * sin(phi_v);
      start[0] = pos[0] + min_ray_length * cos(phi_v) * cos(yaw + phi_h);
      start[1] = pos[1] + min_ray_length * cos(phi_v) * sin(yaw + phi_h);
      start[2] = pos[2] + min_ray_length * sin(phi_v);
      // Do raycasting to check info gain
      Eigen::Vector3i idx;
      bool in_pred_box = false;
      int local_idx;
      bool flag_unknown = false;
      raycaster_->input(start, end);
      while (raycaster_->nextId(idx)) {
        // Idx is global idx in map, check occupancy first
        // If hit obstacle, break
        if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
          break;
        // Cell is unknown in the original map
        if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          // New way:
          // First check if cell in local pred map
          in_pred_box = globalInPredBox(idx, buf_id);
          if (!in_pred_box) {
            // Not in pred map, count gain assuming unknown -> gain
            gain ++;
            continue;
          };
          globalToPredIdx(idx, local_idx, buf_id);
          // If pred is unknown
          if (all_pred_buffer_[buf_id][local_idx] < edt_env_->sdf_map_->mp_->clamp_min_log_ - 1e-3) {
            gain ++;
          }
          // If pred is occupied
          else if (all_pred_buffer_[buf_id][local_idx] > edt_env_->sdf_map_->mp_->min_occupancy_log_) {
            gain ++;
            break;
          }
          // If pred is free
          else gain ++;
        }
        // If cell is free, continue
      }
    }
    slice_gains.push_back(gain);
  }

  // Step 2: Sum up slices' gain to compute gain for each yaw of the sensor
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double sensing_yaw = yaw - M_PI_2 + M_PI / 9.0 * i + half_h_fov;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(sensing_yaw, gain));
  }

  // Step 3: find best yaw
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const pair<double, int>& p1, const pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 1; ++i) {
    if (yaw_gains[i].second < min_info_gain_) {
      // std::cout << "smaller than min info gain" << std::endl;
      break;
    } 
    Viewpoint vp = {pos, yaw_gains[i].first, 0, yaw_gains[i].second };
    wrapYaw(vp.yaw_);
    vps.push_back(vp);
  }
}


void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  // Downsample value=3 in config
  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
      // if (visited_flag_[adr] == 0)
      // {
      //   init_que.push(nbr);
      //   visited_flag_[adr] = 1;
      // }
    }
  }
  return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->isInMap(idx);
}


bool FrontierFinder::globalInPredBox(const Eigen::Vector3i& global_id, const int& buf_id) {
  Eigen::Vector3i local_box_min = all_local_box_id_min_[buf_id];
  Eigen::Vector3i local_box_max = all_local_box_id_max_[buf_id];
  if (global_id(0) < local_box_min(0) || 
      global_id(1) < local_box_min(1) || 
      global_id(2) < local_box_min(2)) return false;
  if (global_id(0) > local_box_max(0) || 
      global_id(1) > local_box_max(1) ||
      global_id(2) > local_box_max(2)) return false;
  return true;
}


void FrontierFinder::globalToPredId(const Eigen::Vector3i& global_id, Eigen::Vector3i& local_id, const int& buf_id) {
  Eigen::Vector3i local_box_min = all_local_box_id_min_[buf_id];
  local_id = global_id - local_box_min;
}

void FrontierFinder::globalToPredIdx(const Eigen::Vector3i& global_id, int& local_idx, const int& buf_id) {
  Eigen::Vector3i local_id;
  globalToPredId(global_id, local_id, buf_id);
  Eigen::Vector3i box_size;
  Eigen::Vector3i local_box_min = all_local_box_id_min_[buf_id];
  Eigen::Vector3i local_box_max = all_local_box_id_max_[buf_id];
  box_size = local_box_max - local_box_min;
  local_idx = local_id(0) * box_size(1) * box_size(2) +
              local_id(1) * box_size(2) +
              local_id(2);
}

void FrontierFinder::setCurrTopoId(int current_topo_id) {
  current_topo_node_id_ = current_topo_id;
}


bool FrontierFinder::detectHorizontalGap(const Vector3d& pos, Vector3d& gap_pos, double& gap_yaw, 
                                          const double& radius, const double& h_top, 
                                          const double& h_bot, const bool save, const bool confirm) {
  std::vector<uint8_t> occ_buf;
  Vector3d local_range_min, local_range_max;

  double res = edt_env_->sdf_map_->getResolution();
  int side_size = radius * 2 / res;
  int occ_buf_size = std::pow(side_size, 2);

  occ_buf.resize(occ_buf_size);
  fill(occ_buf.begin(), occ_buf.end(), 0);

  local_range_min(0) = pos(0) - radius;
  local_range_min(1) = pos(1) - radius;
  local_range_min(2) = h_bot;
  local_range_max(0) = pos(0) + radius;
  local_range_max(1) = pos(1) + radius;
  local_range_max(2) = h_top;

  Eigen::Vector3i box_min_id, box_max_id, box_size;
  edt_env_->sdf_map_->posToIndex(local_range_min, box_min_id);
  edt_env_->sdf_map_->posToIndex(local_range_max, box_max_id);
  box_size = box_max_id - box_min_id;
  // For mem safe operation
  if (box_size(0) != side_size || box_size(1) != side_size) {
    box_size(0) = side_size;
    box_size(1) = side_size;
    box_max_id(0) = box_min_id(0) + side_size;
    box_max_id(1) = box_min_id(1) + side_size;
  }
  // Copy from occupancy buffer to local buffer.
  int local_idx;
  Eigen::Vector3i global_id;
  Eigen::Vector3d global_pos;
  bool in_map;
  uint8_t occ_value;
  for (int id_x = box_min_id(0); id_x < box_max_id(0); id_x++) {
    for (int id_y = box_min_id(1); id_y < box_max_id(1); id_y++) {
      occ_value = 0;
      local_idx = (id_x - box_min_id(0)) * box_size(1) + (id_y - box_min_id(1));
      for (int id_z = box_min_id(2); id_z < box_max_id(2); id_z++) {
        global_id(0) = id_x;
        global_id(1) = id_y;
        global_id(2) = id_z;
        in_map = edt_env_->sdf_map_->isInMap(global_id);
        if (in_map) {
          int occ = edt_env_->sdf_map_->getOccupancy(global_id);
          if (occ == SDFMap::OCCUPIED) {
            occ_value = 255;
            break;
          }
        }
      } 
      occ_buf[local_idx] = occ_value;
    }
  }


  cv::Mat img_from_occ(side_size, side_size, CV_8U, occ_buf.data());
  cv::Mat edges;

  // Probabilistic Line Transform
  std::vector<cv::Vec4i> lines_p; // will hold the results of the detection
  double rho = 0.5; // distance resolution in pixels of the Hough grid
  double theta = CV_PI / 180;  // angular resolution in radians of the Hough grid

  if (confirm) {
    cv::Canny(img_from_occ, edges, 50, 150);
    cv::HoughLinesP(edges, lines_p, rho, theta, min_vote_, h_min_line_length_, h_max_line_gap_); // runs the actual detection
  } else {
    // Edge detection
    cv::Canny(img_from_occ, edges, 50, 150);
    cv::HoughLinesP(edges, lines_p, rho, theta, min_vote_, h_min_line_length_, h_max_line_gap_); // runs the actual detection

  }

  // Detect doors
  // 1. All parallel lines sit on one straight line
  double x1, y1, x2, y2, x3, y3, x4, y4;
  double cos_angle, cos3, cos4;
  double para_small, para_big, thres_s, thres_b;
  para_small = 20;
  para_big = 180 - para_small;
  thres_s = cos(para_small * CV_PI / 180);
  thres_b = cos(para_big * CV_PI / 180);

  std::vector<std::vector<int>> pairs;
  for (int i = 0; i < lines_p.size(); i++) {
    for (int j = i+1; j < lines_p.size(); j++) {
      x1 = lines_p[i][2] - lines_p[i][0];
      y1 = lines_p[i][3] - lines_p[i][1];
      x2 = lines_p[j][2] - lines_p[j][0];
      y2 = lines_p[j][3] - lines_p[j][1];
      cos_angle = (x1 * x2 + y1 * y2) / (sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2));
      if (cos_angle > thres_s || cos_angle < thres_b) {
        x3 = lines_p[j][0] - lines_p[i][0];
        y3 = lines_p[j][1] - lines_p[i][1];
        cos3 = (x1 * x3 + y1 * y3) / (sqrt(x1*x1 + y1*y1) * sqrt(x3*x3 + y3*y3));
        x4 = lines_p[j][2] - lines_p[i][2];
        y4 = lines_p[j][3] - lines_p[i][3];
        cos4 = (x1 * x4 + y1 * y4) / (sqrt(x1*x1 + y1*y1) * sqrt(x4*x4 + y4*y4));
        
        if ((cos3 > thres_s || cos3 < thres_b) && (cos4 > thres_s || cos4 < thres_b)) {
          pairs.push_back({i, j});
        }
      }
    }
  }

  // 2. Find door
  std::vector<int> pt1, pt2, pt3, pt4;
  std::vector<std::vector<std::vector<int>>> all_pt_pairs;
  std::vector<double> all_dis;
  bool has_door = false;
  double door_x, door_y;
  double door_dir_x, door_dir_y, norm;
  std::vector<std::pair<double, double>> door_vec;
  for (auto pair: pairs) {
    pt1 = {lines_p[pair[0]][0], lines_p[pair[0]][1]};
    pt2 = {lines_p[pair[0]][2], lines_p[pair[0]][3]};
    pt3 = {lines_p[pair[1]][0], lines_p[pair[1]][1]};
    pt4 = {lines_p[pair[1]][2], lines_p[pair[1]][3]};
    all_pt_pairs = {{pt1, pt3}, {pt1, pt4}, {pt2, pt3}, {pt2, pt4}};
    all_dis.clear();
    for (auto pt_pair: all_pt_pairs) {
      all_dis.push_back(sqrt( pow(pt_pair[0][0] - pt_pair[1][0], 2) + pow(pt_pair[0][1] - pt_pair[1][1], 2)));
    }
    
    auto min_it = std::min_element(all_dis.begin(), all_dis.end());
    int idx = min_it - all_dis.begin();

    if (h_min_door_ < *min_it < h_max_door_) {
      door_x = (all_pt_pairs[idx][0][0] + all_pt_pairs[idx][1][0]) / 2;
      door_y = (all_pt_pairs[idx][0][1] + all_pt_pairs[idx][1][1]) / 2;
      door_dir_x = all_pt_pairs[idx][1][1] - all_pt_pairs[idx][0][1];
      door_dir_y = all_pt_pairs[idx][0][0] - all_pt_pairs[idx][1][0];
      has_door = true;
      break;
    }
  }
  if (has_door) {
    Vector3d door_pos = {pos(0) - (side_size / 2) * 0.1 + door_y * 0.1, 
                         pos(1) - (side_size / 2) * 0.1 + door_x * 0.1, 
                         0.0};
    // Correct door yaw pointing to outward
    double door_yaw = atan2(door_dir_y, door_dir_x) + M_PI / 2;
    double line_yaw = atan2(cur_pos_(1) - door_pos(1), cur_pos_(0) - door_pos(0));
    wrapYaw(door_yaw);
    vector<double> ideal_door_yaw = {-M_PI, M_PI, 0, -M_PI/2, M_PI/2};
    int arg_cloest = -1;
    double cloest = 4*M_PI;
    for (int i = 0; i < 4; i++) {
      double tmp = abs(door_yaw - ideal_door_yaw[i]);
      if (tmp < cloest) {
        cloest = tmp;
        arg_cloest = i;
      }
    }
    door_yaw = ideal_door_yaw[arg_cloest];

    wrapYaw(line_yaw);
    if (abs(angles::shortest_angular_distance(door_yaw, line_yaw)) > M_PI/2) {
      door_yaw = door_yaw + M_PI;
      wrapYaw(door_yaw);
    }
    gap_pos = door_pos;
    gap_yaw = door_yaw;
    return true;
  } else {
    return false;
  }
}


bool FrontierFinder::detectVerticalGap(Vector3d& gap_pos, double& gap_yaw, 
                                      const double& height, const double& width, const double& thick, const bool save, const bool confirm) {
  // Step 0: extract local buf

  std::vector<uint8_t> occ_buf;
  Vector3d local_range_min, local_range_max;
  double res = edt_env_->sdf_map_->getResolution();
  int side_size1 =  (height-0.6) / res;
  int side_size2 =  width / res;
  int occ_buf_size = side_size1 * side_size2;
  occ_buf.resize(occ_buf_size);
  fill(occ_buf.begin(), occ_buf.end(), 0);

  // If gap pos unknown, return false
  Vector3d tmp_gap_pos = gap_pos;
  tmp_gap_pos(2) = cur_pos_(2);
  int tmp_gap_occ = edt_env_->sdf_map_->getOccupancy(tmp_gap_pos);
  if (tmp_gap_occ == SDFMap::UNKNOWN) return false;

  vector<double> angles = {-M_PI, M_PI, 0, -M_PI/2, M_PI/2};
  int arg_cloest = -1;
  double cloest = 4*M_PI;
  for (int i = 0; i < 4; i++) {
    double tmp = abs(gap_yaw - angles[i]);
    if (tmp < cloest) {
      cloest = tmp;
      arg_cloest = i;
    }
  }
  if (arg_cloest <= 2) {
    local_range_min(0) = gap_pos(0) - thick/2;
    local_range_min(1) = gap_pos(1) - width/2;
    local_range_min(2) = 0.6;
    local_range_max(0) = gap_pos(0) + thick/2;
    local_range_max(1) = gap_pos(1) + width/2;
    local_range_max(2) = height;
    Eigen::Vector3i box_min_id, box_max_id, box_size;
    edt_env_->sdf_map_->posToIndex(local_range_min, box_min_id);
    edt_env_->sdf_map_->posToIndex(local_range_max, box_max_id);
    box_size = box_max_id - box_min_id;
    // For mem safe operation
    if (box_size(1) != side_size2 || box_size(2) != side_size1) {
      box_size(1) = side_size2;
      box_size(2) = side_size1;
      box_max_id(1) = box_min_id(1) + side_size2;
      box_max_id(2) = box_min_id(2) + side_size1;
    }
    // Copy from occupancy buffer to local buffer.
    int local_idx;
    Eigen::Vector3i global_id;
    Eigen::Vector3d global_pos;
    bool in_map;
    uint8_t occ_value;
    for (int id_y = box_min_id(1); id_y < box_max_id(1); id_y++) {
      for (int id_z = box_min_id(2); id_z < box_max_id(2); id_z++) {
        occ_value = 0;
        local_idx = (id_y - box_min_id(1)) * box_size(2) + (id_z - box_min_id(2));
        for (int id_x = box_min_id(0); id_x < box_max_id(0); id_x++) {
          global_id(0) = id_x;
          global_id(1) = id_y;
          global_id(2) = id_z;
          in_map = edt_env_->sdf_map_->isInMap(global_id);
          if (in_map) {
            int occ = edt_env_->sdf_map_->getOccupancy(global_id);
            if (occ == SDFMap::OCCUPIED) {
              occ_value = 255;
              break;
            }
          }
        } 
        occ_buf[local_idx] = occ_value;
      }
    }


    
    // Step 1: Check center region occupancy
    int center_idx = (box_max_id(1) - box_min_id(1)) / 2 * box_size(2) + (box_max_id(2) - box_min_id(2)) / 2;

    if (occ_buf[center_idx] == 255) {
      ROS_WARN("[DetectVerticalGap] Center occupied");
      return false;
    }
  } else {
    local_range_min(0) = gap_pos(0) - width/2;
    local_range_min(1) = gap_pos(1) - thick/2;
    local_range_min(2) = 0.4;
    local_range_max(0) = gap_pos(0) + width/2;
    local_range_max(1) = gap_pos(1) + thick/2;
    local_range_max(2) = height;
    Eigen::Vector3i box_min_id, box_max_id, box_size;
    edt_env_->sdf_map_->posToIndex(local_range_min, box_min_id);
    edt_env_->sdf_map_->posToIndex(local_range_max, box_max_id);
    box_size = box_max_id - box_min_id;
    // For mem safe operation
    if (box_size(0) != side_size2 || box_size(2) != side_size1) {
      box_size(0) = side_size2;
      box_size(2) = side_size1;
      box_max_id(0) = box_min_id(0) + side_size2;
      box_max_id(2) = box_min_id(2) + side_size1;
    }
    // Copy from occupancy buffer to local buffer.
    int local_idx;
    Eigen::Vector3i global_id;
    Eigen::Vector3d global_pos;
    bool in_map;
    uint8_t occ_value;
    for (int id_x = box_min_id(0); id_x < box_max_id(0); id_x++) {
      for (int id_z = box_min_id(2); id_z < box_max_id(2); id_z++) {
        occ_value = 0;
        local_idx = (id_x - box_min_id(0)) * box_size(2) + (id_z - box_min_id(2));
        for (int id_y = box_min_id(1); id_y < box_max_id(1); id_y++) {
          global_id(0) = id_x;
          global_id(1) = id_y;
          global_id(2) = id_z;
          in_map = edt_env_->sdf_map_->isInMap(global_id);
          if (in_map) {
            int occ = edt_env_->sdf_map_->getOccupancy(global_id);
            if (occ == SDFMap::OCCUPIED) {
              occ_value = 255;
              break;
            }
          }
        } 
        occ_buf[local_idx] = occ_value;
      }
    }

    // Step 1: Check center pixel occupancy
    int center_idx = (box_max_id(0) - box_min_id(0)) / 2 * box_size(2) + (box_max_id(2) - box_min_id(2)) / 2;
    if (occ_buf[center_idx] == 255) {
      ROS_WARN("[DetectVerticalGap] Center occupied");
      return false;
    }
  }
  // Step 2: convert to img, detect edge and lines

  cv::Mat img_from_occ(side_size1, side_size2, CV_8U, occ_buf.data());
  cv::Mat edges;
  // // Add Blur
  // cv::GaussianBlur(img_from_occ, edges, cv::Size(5, 5), 0);
  // // Add thres
  // cv::threshold(edges, edges, 90, 255, cv::THRESH_BINARY);
  // // Edge detection
  cv::Canny(img_from_occ, edges, 50, 150, 3);
  // Probabilistic Line Transform
  std::vector<cv::Vec4i> lines_p; // will hold the results of the detection
  double rho = 1; // distance resolution in pixels of the Hough grid
  double theta = CV_PI / 180;  // angular resolution in radians of the Hough grid

  cv::HoughLinesP(edges, lines_p, rho, theta, min_vote_, v_min_line_length_, v_max_line_gap_); // runs the actual detection

  // Step 3: find parallel lines
  double x1, y1, x2, y2, x3, y3, x4, y4;
  double cos_angle, cos3, cos4;
  double para_small, para_big, thres_s, thres_b;
  para_small = 20;
  para_big = 180 - para_small;
  thres_s = cos(para_small * CV_PI / 180);
  thres_b = cos(para_big * CV_PI / 180);
  double ux1, uy1, vx5, vy5, vx6, vy6;
  double d1, d2, dis, dis_thres_s, dis_thres_b, min_dis;

  min_dis = 100;
  for (int i = 0; i < lines_p.size(); i++) {
    for (int j = i+1; j < lines_p.size(); j++) {
      x1 = lines_p[i][2] - lines_p[i][0];
      y1 = lines_p[i][3] - lines_p[i][1];
      x2 = lines_p[j][2] - lines_p[j][0];
      y2 = lines_p[j][3] - lines_p[j][1];
      cos_angle = (x1 * x2 + y1 * y2) / (sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2));
      if (cos_angle > thres_s || cos_angle < thres_b) {
        x3 = lines_p[j][0] - lines_p[i][0];
        y3 = lines_p[j][1] - lines_p[i][1];
        cos3 = (x1 * x3 + y1 * y3) / (sqrt(x1*x1 + y1*y1) * sqrt(x3*x3 + y3*y3));
        x4 = lines_p[j][2] - lines_p[i][2];
        y4 = lines_p[j][3] - lines_p[i][3];
        cos4 = (x1 * x4 + y1 * y4) / (sqrt(x1*x1 + y1*y1) * sqrt(x4*x4 + y4*y4));
        // Parallel but not sit on one line
        if ((cos3 < thres_s || cos3 > thres_b) && (cos4 < thres_s || cos4 > thres_b)) {
          ux1 = x1 / sqrt(x1*x1 + y1*y1);
          uy1 = y1 / sqrt(x1*x1 + y1*y1);
          vx5 = lines_p[j][0] - lines_p[i][0];
          vy5 = lines_p[j][1] - lines_p[i][1];
          vx6 = lines_p[j][2] - lines_p[i][0];
          vy6 = lines_p[j][3] - lines_p[i][1];
          // Distance = || unit cross (x) vec ||;
          d1 = abs(ux1 * vy5 - vx5 * uy1);
          d2 = abs(ux1 * vy6 - vx6 * uy1);
          dis = (d1 + d2) / 2;
          if (dis < min_dis) min_dis = dis;
        }
      }
    }
  }
  if (v_min_door_ < dis < v_max_door_) {
    return true;
  } else return false;
}


void FrontierFinder::detectDoors(const double& radius, 
                                const double& h_top, const double& h_bot, 
                                const double& height, const double& width, 
                                const double& thick) {
  ros::Time t1 = ros::Time::now();
  Vector3d gap_pos;
  double gap_yaw;
  bool is_door = false;
  for (auto it = tmp_frontiers_.begin(); it != tmp_frontiers_.end(); ++it) {
    if (it->type_ == ROOM) continue;
    Vector3d pos = it->average_; 
    if (detectHorizontalGap(pos, gap_pos, gap_yaw, radius, h_top, h_bot, true)) {

      is_door = detectVerticalGap(gap_pos, gap_yaw, height, width, thick, true);
      if (is_door) {


        if (!isDoorExist(gap_pos, gap_yaw)) {
          DoorInfo door;
          door.id_ = ++global_door_id_;
          door.pos_ = gap_pos;
          door.yaw_ = gap_yaw;
          door.visited_ = false;
          door.confirmed_ = false;
          door.confirming_ = false;
          doors_.push_back(door);
        }
      }
    }
  }
  ROS_WARN_THROTTLE(5.0, "Door Detection t: %lf", (ros::Time::now() - t1).toSec());
}

bool FrontierFinder::isDoorExist(const Vector3d& pos, const double& yaw, const double& dis_thres, const double& yaw_thres) {
  // Assume no two doors within thres
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    double dis = pow(iter->pos_(0) - pos(0), 2) + pow(iter->pos_(1) - pos(1), 2) + pow(iter->pos_(2) - pos(2), 2);
    if (dis < dis_thres*dis_thres) return true;

  }
  return false;
}

void FrontierFinder::removeFPDoor(const double& radius, const double& h_top, const double& h_bot, 
                                  const double& height, const double& width, const double& thick) {
  ros::Time t1 = ros::Time::now();
  auto removeDoor = [&](list<DoorInfo>::iterator& it, list<DoorInfo>& doors) {
    ROS_ERROR("-------------------REMOVE door id: %d", it->id_);
    it = doors.erase(it);
  };
  // For each existing door, double check it
  std::cout << "[removeFPDoor] before remove: " << doors_.size() << std::endl;
  for (auto iter = doors_.begin(); iter != doors_.end();) {
    if (iter->confirmed_ || iter->confirming_) {
      iter++;
      continue;
    }
    Vector3d gap_pos;
    double gap_yaw;
    bool is_door = false;
    if (detectHorizontalGap(iter->pos_, gap_pos, gap_yaw, radius, h_top, h_bot)) {
      is_door = detectVerticalGap(gap_pos, gap_yaw, height, width, thick);
      if (is_door) {
        ++iter;
        continue;
      }
    } 
    removeDoor(iter, doors_);
  }
  std::cout << "[removeFPDoor] after remove: " << doors_.size() << std::endl;
}


void FrontierFinder::removeDoorIfExist() {
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    if (iter->id_ == door_goal_id_) {
      doors_.erase(iter);
      return;
    }
  }
}

void FrontierFinder::removeDoorById(const int& id) {
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    if (iter->id_ == id) {
      doors_.erase(iter);
      return;
    }
  }
}

bool FrontierFinder::confirmDoor(const double& radius, const double& h_top, const double& h_bot, 
                                  const double& height, const double& width, const double& thick) {
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    ROS_ERROR("curr id: %d, goal id: %d", iter->id_, door_goal_id_);
    if (iter->id_ == door_goal_id_) {
      // Confirm door now:
      double is_door = false;
      Vector3d gap_pos;
      double gap_yaw;
      ROS_ERROR("Confirm door: detect horizontal");
      if (detectHorizontalGap(iter->pos_, gap_pos, gap_yaw, radius, h_top, h_bot, true, true)) {
        ROS_ERROR("Confirm door: horizontal true, detect vertical");
        is_door = detectVerticalGap(gap_pos, gap_yaw, height, width, thick, true, true);
      }
      if (is_door) {
        ROS_ERROR("Confirm door: vertical true");
        iter->confirmed_ = true;
        iter->confirming_ = false;
        return true;
      } else {
        ROS_ERROR("Confirm door: false");
        return false;
      }
    } 
  }
  ROS_ERROR("No such a door");
  return false;
}

void FrontierFinder::transitBCState(BEHAVIOR_STATE new_state, std::string pos_call) {
  int pre_s = int(behavior_state_);
  behavior_state_ = new_state;
  cout << "[" + pos_call + "]: from " + behavior_state_str_[pre_s] + " to " + behavior_state_str_[int(new_state)]
       << endl;
}

void FrontierFinder::setCurrPos(const Vector3d& cur_pos, const double& cur_yaw) {
  cur_pos_ = cur_pos;
  cur_yaw_ = cur_yaw;
}


void FrontierFinder::findNearestDoor(int& door_id, double& cost, Vector3d& door_pos, double& door_yaw) {
  double min_cost = COST_INF;
  int min_door_id = -1;
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    if (iter->visited_) continue;
    vector<Vector3d> tmp_path;
    double tmp_cost = ViewNode::computePathCost(cur_pos_, iter->pos_, tmp_path);
    if (tmp_cost < min_cost) {
      min_cost = tmp_cost;
      min_door_id = iter->id_;
      door_pos = iter->pos_;
      door_yaw = iter->yaw_;
    }
  }
  door_id = min_door_id;
  cost = min_cost;
}


bool FrontierFinder::reachDoorConfirmPos() {
  if ((cur_pos_.head<2>() - door_confirm_pos_.head<2>()).norm() < door_confirm_thres_) {
    double yaw_dif = abs(cur_yaw_ - door_confirm_yaw_);
    wrapYaw(yaw_dif);
    if (yaw_dif < M_PI / 4) return true;
    else return false;
  } else {
    return false;
  }
}

bool FrontierFinder::reachDoorCrossPos() {
  if ((cur_pos_ .head<2>()- door_cross_pos_.head<2>()).norm() < door_cross_thres_) {
    double yaw_dif = abs(cur_yaw_ - door_cross_yaw_);
    wrapYaw(yaw_dif);
    if (yaw_dif < M_PI / 2) return true;
    else return false;
  } else {
    return false;
  }
}

bool FrontierFinder::reachDoorExitPos() {
  if ((cur_pos_.head<2>() - door_exit_pos_.head<2>()).norm() < door_exit_thres_) {

    return true;
  } else {
    return false;
  }
}


bool FrontierFinder::hasDoors() {
  if (doors_.size() > 0) {
    for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
      if (iter->visited_ == false) {
        return true;
      }
    }
    return false;
  }
  else return false;
}

void FrontierFinder::labelRoomVisited() {
  for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
    if (iter->id_ == door_goal_id_) {
      iter->visited_ = true;
      room_visited_ = true;
      return;
    }
  }
}


bool FrontierFinder::setNextDoor(int n_plan_fails) {
  // Navigate to next door and confirm door
  int nearest_id;
  double nearest_cost;
  Vector3d next_door_pos; 
  double next_door_yaw;
  bool next_door = false;
  while (!next_door) {
    if (hasDoors()) {
      findNearestDoor(nearest_id, nearest_cost, next_door_pos, next_door_yaw);
      if (n_plan_fails > 3) {
        removeDoorById(nearest_id);
        continue;
      }
      // Compute confirm pos:
      Vector3d tmp_conf_pos = next_door_pos + door_confirm_d_ * Vector3d(cos(next_door_yaw), sin(next_door_yaw), 0);
      // tmp_conf_pos(2) = cur_pos_(2);
      tmp_conf_pos(2) = 1.0;
      double tmp_conf_yaw = next_door_yaw + M_PI;
      wrapYaw(tmp_conf_yaw);
      if (isPosValid(tmp_conf_pos)) {
        next_door = true;
        setDoorPos(next_door_pos, next_door_yaw, tmp_conf_pos, tmp_conf_yaw);
        door_goal_set_ = true;
        room_visited_ = false;
        door_goal_id_ = nearest_id;
        for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
          if (iter->id_ == door_goal_id_) {
            iter->confirming_ = true;
          } 
        }
        ROS_ERROR("set door id: %d", door_goal_id_);
        return true;
      } else {
        // search around:
        Vector3d tmp_confirm_pos;
        for (int i = -2; i <= 2; i ++) {
          for (int j = -2; j <=2; j++) {
            for (int k = -2; k <=2; k++) {
              tmp_confirm_pos = tmp_conf_pos + resolution_ * Vector3d(i, j, k);
              if (isPosValid(tmp_confirm_pos)) {
                next_door = true;
                setDoorPos(next_door_pos, next_door_yaw, tmp_confirm_pos, tmp_conf_yaw);
                door_goal_set_ = true;
                room_visited_ = false;
                door_goal_id_ = nearest_id;
                for (auto iter = doors_.begin(); iter != doors_.end(); ++iter) {
                  if (iter->id_ == door_goal_id_) {
                    iter->confirming_ = true;
                  } 
                }
                return true;
              }
            }
          }
        }
        removeDoorById(nearest_id);
        return false;
      }
    } else {
      // No unvisited doors
      return false;
    }
  }
}


// Set both door pos and confirm pos
void FrontierFinder::setDoorPos(const Vector3d& door_pos, const double& door_yaw, const Vector3d& conf_pos, const double& conf_yaw) {
  door_goal_pos_ = door_pos;
  door_goal_pos_(2) = 1.0;
  door_goal_yaw_ = door_yaw;

  door_confirm_pos_ = conf_pos;
  door_confirm_pos_(2) = 1.0;
  door_confirm_yaw_ = conf_yaw;
  wrapYaw(door_confirm_yaw_);
}


void FrontierFinder::getDoorConfirmPos(Vector3d& door_pos, double& door_yaw) {
  door_pos = door_confirm_pos_;
  door_yaw = door_confirm_yaw_;
}

bool FrontierFinder::isPosValid(const Vector3d& door_pos) {
  if (edt_env_->sdf_map_->getInflateOccupancy(door_pos) == 1 || isNearUnknown(door_pos, 0.6) || isNearObstacle(door_pos, 0.6)) {
    return false;
  }
  return true;
}

bool FrontierFinder::getCrossDoorGoal(Vector3d& door_pos, double& door_yaw) {
  Vector3d tmp_cross_pos;
  tmp_cross_pos = door_goal_pos_ - door_cross_d_ * Vector3d(cos(door_goal_yaw_), sin(door_goal_yaw_), 0);
  if (isPosValid(tmp_cross_pos)) {
    door_cross_pos_ = tmp_cross_pos;
    door_cross_yaw_ = door_goal_yaw_ + M_PI;
    // return values:
    door_pos = door_cross_pos_;
    door_yaw = door_cross_yaw_;
    return true;
  } else return false;
}

void FrontierFinder::getExitDoorGoal(Vector3d& door_pos, double& door_yaw) {
  door_exit_pos_ = door_goal_pos_ + door_exit_d_ * Vector3d(cos(door_goal_yaw_), sin(door_goal_yaw_), 0);
  door_exit_yaw_ = door_goal_yaw_;
  // return values:
  door_pos = door_exit_pos_;
  door_yaw = door_exit_yaw_;
}

int FrontierFinder::occSearch(const Vector3d& pos) {
  return edt_env_->sdf_map_->getOccupancy(pos);
}


}  // namespace bg_planner