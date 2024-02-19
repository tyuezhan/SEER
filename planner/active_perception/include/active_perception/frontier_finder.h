#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>
#include <angles/angles.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "occ_predictor_msgs/PredictPCL.h"

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace bg_planner {
class EDTEnvironment;
class PerceptionUtils;
#define COST_INF 100000;

enum BEHAVIOR_STATE {IN_CORRIDOR, NAV_TO_DOOR, IN_ROOM, EXIT_ROOM};
enum FTR_TYPE {CORRIDOR, ROOM};
// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
  int info_gain_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // FUEL: Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;

  // Corresponding global buffer id for the frontier cluster
  int buf_id_;
  // Whether the frontier cluster has prediction
  bool has_pred_;
  // Semantic label of the frontier cluster
  int type_;
  // Corresponding global id, used in topological node
  int topo_id_;
  // Add utility to frontier viewpoint
  // May remove paths and costs as they will not be used
  // int nearest_door_id_;
};

// A node in topological map
struct TopoNode {
  // Topo id is same as the topo_id saved in frontier cluster
  int topo_id_;
  // Parent id, used to track topological parent of the node
  int parent_id_;
  // Semantic label of the topo node
  int type_;
  // save viewpoint pos for visualization
  Vector3d pos_;
};

struct DoorInfo {
  // global id of the detected door
  int id_;
  // position of the door
  Vector3d pos_;
  // Yaw of the door (normal)
  double yaw_;
  // visited
  bool visited_;
  // Confirmed
  bool confirmed_;
  bool confirming_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiersBC();
  void predOnFrontiers();
  void computeFrontiersToVisitBC();

  void makeFrontierBuffers(list<Frontier>& frontiers);
  void makeTopoNode(Frontier& ftr);


  //get frontier
  void getFrontierObjs(list<Frontier> &cluster_objs);
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  void getFrontierPred(vector<vector<Vector3d>>& preds);
  void getDoors(vector<Vector3d>& pos, vector<double>& yaws);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsBC(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws, 
                            vector<Eigen::Vector3d>& averages, vector<int>& info_gains, vector<int>& topo_ids);


  void recheckViewpoints();

  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);
  
  void setCurrTopoId(int current_topo_id);
  void setCurrPos(const Vector3d& cur_pos, const double& cur_yaw);

  // Behavior control
  void detectDoors(const double& radius=3.5, const double& h_top=1.3, const double& h_bot=0.8, 
                  const double& height=1.8, const double& width=1.6, const double& thick=0.6);
  void removeFPDoor(const double& radius=3.0, const double& h_top=1.3, const double& h_bot=0.8, 
                    const double& height=2.0, const double& width=1.5, const double& thick=0.6);
  bool confirmDoor(const double& radius=3.0, const double& h_top=1.3, const double& h_bot=0.8, 
                  const double& height=2.0, const double& width=1.5, const double& thick=0.6);
  bool hasDoors();
  bool setNextDoor(int n_plan_fails);
  void setDoorPos(const Vector3d& door_pos, const double& door_yaw, const Vector3d& conf_pos, const double& conf_yaw);

  bool reachDoorConfirmPos();
  bool reachDoorCrossPos();
  bool reachDoorExitPos();

  void removeDoorIfExist();
  void removeDoorById(const int& id);
  void getDoorConfirmPos(Vector3d& door_pos, double& door_yaw);
  bool isPosValid(const Vector3d& door_pos);
  bool getCrossDoorGoal(Vector3d& door_pos, double& door_yaw);
  void getExitDoorGoal(Vector3d& door_pos, double& door_yaw);
  void labelRoomVisited();
  void labelRoomFtr(const double& d=0.8);
  bool hasRoomFtr();

  int computeYawInfoGain(const Eigen::Vector3d& pos, 
                         const double& yaw);

  bool room_visited_;
  
  // Exploration behavior control
  void transitBCState(BEHAVIOR_STATE new_state, std::string pos_call);

  shared_ptr<PerceptionUtils> percep_utils_;
  
  BEHAVIOR_STATE behavior_state_;
  vector<std::string> behavior_state_str_;

  int occSearch(const Vector3d& pos);

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpointsIG(Frontier& frontier);



  void computeInfoGain(const Eigen::Vector3d& pos, const double& yaw, const int& buf_id, vector<Viewpoint>& vps);
  void computeInfoGainNoPred(const Eigen::Vector3d& pos, const double& yaw, const int& buf_id, vector<Viewpoint>& vps);
  bool isNearUnknown(const Vector3d& pos, double clearance=-1);
  bool isNearObstacle(const Vector3d& pos, double clearance=-1);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Global map and local prediction map idx conversion
  bool globalInPredBox(const Eigen::Vector3i& global_id, const int& buf_id);
  void globalToPredId(const Eigen::Vector3i& global_id, Eigen::Vector3i& local_id, const int& buf_id);
  void globalToPredIdx(const Eigen::Vector3i& global_id, int& local_idx, const int& buf_id);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);

  // Behavior control
  // Detect Doors
  // Detect horizon gap, if found gap of door size, return true with the pos and yaw
  bool detectHorizontalGap(const Vector3d& pos, Vector3d& gap_pos, double& gap_yaw, 
                           const double& radius=4.0, const double& h_top=1.0, const double& h_bot=0.6, const bool save=false, const bool confirm=false);
  // Take a detected gap pos and yaw, try to confirm it's a door size gap vertically, return true/false
  bool detectVerticalGap(Vector3d& gap_pos, double& gap_yaw, 
                         const double& height=1.6, const double& width=1.6, const double& thick=0.4, const bool save=false, const bool confirm=false);

  bool isDoorExist(const Vector3d& pos, const double& yaw, const double& dis_thres=2.0, const double& yaw_thres=40);

  void findNearestDoor(int& door_id, double& cost, Vector3d& door_pos, double& door_yaw);



  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_, room_min_candidate_dist_,
      min_candidate_clearance_, min_candidate_obs_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  int min_info_gain_;
  int pred_size_x_, pred_size_y_, pred_size_z_;
  double v_fov_, h_fov_, min_ray_length_, max_ray_length_;

  // BC Params
  double h_min_line_length_, h_max_line_gap_; // horizontal detect
  double v_min_line_length_, v_max_line_gap_; // vertical detect
  int h_min_door_, h_max_door_, v_min_door_, v_max_door_;


  std::map<int, std::vector<int>> all_local_occ_buffer_;
  std::map<int, std::vector<double>> all_pred_buffer_;
  std::map<int, std::vector<Vector3d>> all_global_pos_buffer_;
  std::map<int, Eigen::Vector3i> all_local_box_id_min_;
  std::map<int, Eigen::Vector3i> all_local_box_id_max_;
  // std::map<int, std::vector<int>> all_local_idx_buffer_;

  std::map<int, TopoNode> topo_nodes_;
  list<DoorInfo> doors_;
  int global_door_id_;
  int global_topo_node_id_;
  int global_ftr_buffer_id_;
  int current_topo_node_id_;
  int ftr_counter_, h_ftr_counter_;

  Vector3d cur_pos_;
  double cur_yaw_;
  bool door_goal_set_;
  Vector3d door_goal_pos_, door_confirm_pos_, door_cross_pos_, door_exit_pos_;
  double door_goal_yaw_, door_confirm_yaw_, door_cross_yaw_, door_exit_yaw_;
  double door_confirm_thres_, door_cross_thres_, door_exit_thres_;
  double door_confirm_d_, door_cross_d_, door_exit_d_;
  int door_goal_id_;
  int min_vote_;

  bool exp_3d_;
  bool has_room_ftr_;
  

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
  
  // ServiceClient for map prediction
  ros::ServiceClient pred_client_;


};

}  // namespace bg_planner
#endif