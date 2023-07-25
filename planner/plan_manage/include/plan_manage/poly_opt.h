#ifndef _POLY_OPT_H_
#define _POLY_OPT_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include <unordered_map>
#include <traj_utils/traj_min_jerk.hpp>
#include <traj_utils/traj_min_yaw.hpp>
#include <traj_utils/lbfgs.hpp>
#include <traj_utils/math.h>
#include <traj_utils/plan_container.hpp>

using std::vector;

namespace bg_planner
{
  
  /*** feasibility checking ***/
  enum CHECKER_TYPE
  {
    VEL_INFI = 1,
    ACC_INFI = 2,
    STA_INFI = 3,
    DYN_INFI = 4,
    FEASIBLE = 5
  };
  

  class PolySolver{  
  public: 

    bool minJerkTrajOpt(Eigen::MatrixXd &inner_pts, // (3, N + 1)
                        Eigen::VectorXd &allo_ts,
                        Eigen::MatrixXd &iS, 
                        Eigen::MatrixXd &fS,
                        std::vector<Eigen::MatrixXd> &hPolys);

    Eigen::VectorXd allocateTime(const Eigen::MatrixXd &wayPs,
                          double vel,
                          double acc);
    
    static double objCallback(void *ptrObj,
                              const double *x,
                              double *grad,
                              const int n);

    inline void init(Eigen::VectorXd wts, 
                     Eigen::VectorXd bds){
      w_ = wts;  // w_time_,  w_vel_,  w_acc_,  w_sta_obs_
      b_ =  bds;
      bb_ = 0.8 * bds;
      jerkOpt_.setWeights(w_, b_);
    }

    void getTraj(min_jerk::Trajectory &traj){traj = minJerkTraj_;}
    min_jerk::Trajectory getTraj(){return minJerkTraj_;}



    min_jerk::JerkOpt jerkOpt_;

  private:

    min_jerk::Trajectory minJerkTraj_;
    Eigen::MatrixXd alloT_, initS_, finalS_;
    std::vector<Eigen::MatrixXd> SFCs_;
    int iter_num_, piece_num_, var_num_, ego_id_;
    Eigen::VectorXd w_, b_, bb_;
    double t_coeff_ = 1.1;
    int isFeasibile();

    void locateViolateStb(std::vector<int> &idxs);
    void locateViolateDyn(std::vector<int> &idxs);

    template <typename EIGENVEC>
    void VT2RT(Eigen::VectorXd &RT, const EIGENVEC &VT);

    template <typename EIGENVEC>
    void RT2VT(const Eigen::VectorXd &RT, EIGENVEC &VT);

  public: 
    typedef std::unique_ptr<PolySolver> Ptr;

  };


  class PolyYawSolver{
    
  public: 

    bool minJerkYawOpt(Eigen::MatrixXd &inner_pts, // (3, N + 1)
                       Eigen::VectorXd &allo_ts,
                       Eigen::MatrixXd &yawState,
                       bool use_ref);

    static double objCallback(void *ptrObj,
                              const double *x,
                              double *grad,
                              const int n);

    void getTraj(min_yaw_jerk::Trajectory &traj){traj = minJerkYawTraj_;}
    double t_coeff_ = 1.2;
    min_yaw_jerk::JerkOpt yawOpt_;

    inline void init(Eigen::VectorXd wts, 
                     Eigen::VectorXd bds){
      w_ = wts;  //w_ref, w_vel_,  w_acc_, 
      b_ = bds;
      bb_ = 0.3 * bds;
      yawOpt_.setWeights(w_, b_);
    }

  private:

    int isFeasibile();
    void setInit();
    void refineInit(int checker);
    void splitSegments(const std::vector<int> idxs,
                       double time_coeff);
                       
    min_yaw_jerk::Trajectory minJerkYawTraj_, minInitTraj_;
    
    Eigen::MatrixXd innerP_;
    Eigen::VectorXd alloT_;
    Eigen::MatrixXd initS_, finalS_;
    Eigen::VectorXd w_, b_, bb_;

    int iter_num_, piece_num_, var_num_;

  public: 
    typedef std::unique_ptr<PolyYawSolver> Ptr;

  };
  
} // namespace

#endif
