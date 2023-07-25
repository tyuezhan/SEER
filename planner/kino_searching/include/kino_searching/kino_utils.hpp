#ifndef _KINO_UTILS_H
#define _KINO_UTILS_H

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <math.h>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <utility>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/root_finder.hpp>
#include <traj_utils/traj_min_jerk.hpp>
#include <traj_utils/math.h>

namespace bg_planner {

  #define IN_CLOSE_SET 'a'
  #define IN_OPEN_SET 'b'
  #define NOT_EXPAND 'c'
  #define inf 1 >> 30

  enum KINO_SEARCH_RESULT 
  { 
    REACH_HORIZON = 1, 
    REACH_END = 2,  
    NO_PATH = 3, 
    REACH_END_BUT_SHOT_FAILS = 4
  };

  struct KinoSearchParameters
  {
    /* search */
    double max_tau_ = 0.25;
    double init_max_tau_ = 0.5;
    double max_vel_  = 3.0;
    double max_acc_  = 3.0;
    double max_jerk_ = 2.0;

    double horizon_ = 7.5;
    double lambda_heu_ = 10.0;

    double w_time_ = 10.0;

    int vis_check_num_ = 10;
    int allocate_num_ = 100000;

    double resolution_, time_resolution_;

  };


  template <typename T>
  void retrievePath(T end_node, std::vector<T> &nodes)
  {
    T cur_node = end_node;
    nodes.clear();
    nodes.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      nodes.push_back(cur_node);
    }

    reverse(nodes.begin(), nodes.end());
  }


  class AccNode {
  public:
    /* -------------------- */
    Eigen::Vector3i index;  // sample jerk
    Eigen::Matrix<double, 6, 1> state;  // pos, vel, acc

    /* yaw dynamics primitves*/ 
    Eigen::Vector2d yaw_state;
    double yaw_input; // ddd yaw
    int yaw_idx;

    double g_score, f_score;
    Eigen::Vector3d input;
    double duration;
    double time;  // dyn
    int time_idx;

    AccNode* parent;
    char node_state;

    double rough_len;

    /* -------------------- */
    AccNode() {
      parent = NULL;
      node_state = NOT_EXPAND;
    } 
    ~AccNode(){};
  };
  typedef AccNode* AccNodePtr;


  class JerkNode {
  public:
    /* -------------------- */
    Eigen::Vector3i index;  // sample jerk
    Eigen::Matrix<double, 9, 1> state;  // pos, vel, acc

    
    /* yaw dynamics primitves*/ 
    Eigen::Vector2d yaw_state;
    double yaw_input; // ddd yaw
    int yaw_idx;


    double g_score, f_score;
    Eigen::Vector3d input;
    double duration;
    double time;  // dyn
    int time_idx;

    JerkNode* parent;
    char node_state;

    /* -------------------- */
    JerkNode() {
      parent = NULL;
      node_state = NOT_EXPAND;
    } 
    ~JerkNode(){};
  };
  typedef JerkNode* JerkNodePtr;


  /*** yaw planning ***/
  class YawNode {
  public:

    /* yaw dynamics primitves*/ 
    Eigen::Vector2d yaw_state;
    double yaw_input; // ddd yaw
    int yaw_idx;
    int path_idx;

    double f_score;
    double duration;
    double time;  // dyn
    int time_idx;

    YawNode* parent;
    /* -------------------- */
    YawNode() {
      parent = NULL;
    } 
    ~YawNode(){};
  };
  typedef YawNode* YawNodePtr;


  template <class T>
  class NodeComparator {
  public:
    bool operator()(T node1, T node2) {
      return node1->f_score > node2->f_score;
    }
  };

  template <typename T>
  struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
      size_t seed = 0;
      for (size_t i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };


  template <class T>
  class NodeHashTable {
  private:
    /* data */
    typedef Eigen::Matrix<int, 5, 1> Vector5i;

    std::unordered_map<Eigen::Vector3i, T, matrix_hash<Eigen::Vector3i>> data_3d_;
    std::unordered_map<Eigen::Vector4i, T, matrix_hash<Eigen::Vector4i>> data_4d_; // with yaw or time
    std::unordered_map<Vector5i, T, matrix_hash<Vector5i>> data_5d_; // with yaw and time

  public:
    NodeHashTable(/* args */) {
    }
    ~NodeHashTable() {
    }
    void insert(Eigen::Vector3i idx, T node) {
      data_3d_.insert(std::make_pair(idx, node));
    }
    void insert(Eigen::Vector3i idx, int yt_idx, T node) {
      data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), yt_idx), node));
    }
    void insert(Eigen::Vector3i idx, int time_idx, int yaw_idx, T node) {
      data_5d_.insert(std::make_pair(Vector5i(idx(0), idx(1), idx(2), time_idx, yaw_idx), node));
    }


    T find(Eigen::Vector3i idx) {
      auto iter = data_3d_.find(idx);
      return iter == data_3d_.end() ? NULL : iter->second;
    }

    T find(Eigen::Vector3i idx, int yt_idx) {
      auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), yt_idx));
      return iter == data_4d_.end() ? NULL : iter->second;
    }

    T find(Eigen::Vector3i idx, int time_idx, int yaw_idx) {
      auto iter = data_5d_.find(Vector5i(idx(0), idx(1), idx(2), time_idx, yaw_idx));
      return iter == data_5d_.end() ? NULL : iter->second;
    }


    void clear() {
      data_3d_.clear();
      data_4d_.clear();
      data_5d_.clear();
    }

  };


template <class T>
class YawHashTable {
private:
  /* data */

public:

  std::unordered_map<int, T> data_;

  YawHashTable(/* args */) {
  }
  ~YawHashTable() {
  }
  void insert(int idx, T node) {
    data_.insert(std::make_pair(idx, node));
  }

  T find(int idx) {
    auto iter = data_.find(idx);
    return iter == data_.end() ? NULL : iter->second;
  }

  void clear() {
    data_.clear();

  }

  int size(){

    return data_.size();
  }


};


}  // namespace 

#endif
