#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>

#include <iostream>

#include <plan_env/obj_predictor.h>

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

using std::vector;
using std::string;
namespace bg_planner {

class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };

  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,
    GRAPH_EDGE = 100,
    RAW_PATH = 200,
    FILTERED_PATH = 300,
    SELECT_PATH = 400
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;       // 0
  ros::Publisher topo_pub_;       // 1
  ros::Publisher predict_pub_;    // 2
  ros::Publisher visib_pub_;      // 3, visibility constraints
  ros::Publisher frontier_pub_;   // 4, frontier searching
  ros::Publisher yaw_pub_;        // 5, yaw trajectory
  ros::Publisher viewpoint_pub_;  // 6, viewpoint planning
  ros::Publisher occ_prediction_pub_; // 7, occupancy prediction
  ros::Publisher door_pub_;       // 8, door
  vector<ros::Publisher> pubs_;   //


  // planning display
  ros::Publisher goal_point_pub;
  ros::Publisher global_list_pub;
  ros::Publisher init_list_pub;
  ros::Publisher optimal_list_pub;
  ros::Publisher a_star_list_pub;
  ros::Publisher guide_vector_pub;
  ros::Publisher intermediate_state_pub;
  ros::Publisher poly_pub;


  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(ros::NodeHandle& nh);

  // new interface
  void fillBasicInfo(visualization_msgs::Marker& mk, const Eigen::Vector3d& scale,
                     const Eigen::Vector4d& color, const string& ns, const int& id, const int& shape);
  void fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list);
  void fillGeometryInfo(visualization_msgs::Marker& mk, const vector<Eigen::Vector3d>& list1,
                        const vector<Eigen::Vector3d>& list2);

  void drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale,
                   const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);
  void drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
                 const string& ns, const int& id, const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                 const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                 const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
                 const string& ns, const int& id, const int& pub_id);
  void drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale, const Eigen::Vector4d& color,
               const string& ns, const int& id, const int& pub_id);

  void drawArrows(const vector<Eigen::Vector3d>& list, const vector<double> yaws, 
                  const double& scale, const Eigen::Vector4d& color, const string& ns, 
                  const int& id, const int& pub_id);
  // Deprecated
  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);
  // draw a polynomial trajectory

  // draw topological graph and paths
  // void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
  //                    const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
  //                    const Eigen::Vector4d& color3, int id = 0);
  // void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  // void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
  void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

  Eigen::Vector4d getColor(const double& h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;


  void drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers);

  void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                          Eigen::Vector4d color, int id);


  void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
  void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);


  void displayInitPathList(Eigen::MatrixXd init_pts, const double scale, int id);
  void displayOptimalList(Eigen::MatrixXd optimal_pts, Eigen::Vector4d color, int id);


  void displayKinoAStarList(std::vector<Eigen::Vector3d> kino_a_star_paths, Eigen::Vector4d color, int id);
  void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
  void displayPolyhedron(visualization_msgs::Marker component, Eigen::Vector4d color, int id);
  std::string frame_id_;
  void displayPolyhedron(decomp_ros_msgs::PolyhedronArray poly_array, int id);

};
}  // namespace bg_planner
#endif