// #include <fstream>
#include <plan_manage/poly_opt.h>
#include <thread>

namespace bg_planner
{

  bool PolySolver::minJerkTrajOpt(Eigen::MatrixXd &inner_pts, //(3, N -1)
                                  Eigen::VectorXd &allo_ts,
                                  Eigen::MatrixXd &iS,
                                  Eigen::MatrixXd &fS,
                                  std::vector<Eigen::MatrixXd> &hPolys)
  {
    

    // set the optimizer
    alloT_ = allo_ts;
    SFCs_ = hPolys;
    initS_ = iS;
    finalS_ = fS;
    piece_num_ = alloT_.size();
    var_num_ = 4 * (piece_num_ - 1) + 1;

    // set the optimizer
    jerkOpt_.reset(initS_, finalS_, piece_num_);
    jerkOpt_.generate(inner_pts, alloT_);

    ros::Time t0 = ros::Time::now(), t1, t2;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-16;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-2;

    /* ---------- optimize ---------- */
    double init_vars[var_num_];
    memcpy(init_vars, inner_pts.data(), inner_pts.size() * sizeof(init_vars[0]));
    Eigen::Map<Eigen::VectorXd> VT(init_vars + inner_pts.size(), alloT_.size());
    RT2VT(alloT_, VT);
    double final_cost = 0;
    iter_num_ = 0;
    t1 = ros::Time::now();
    int result = lbfgs::lbfgs_optimize(var_num_,
                                        init_vars,
                                        &final_cost,
                                        PolySolver::objCallback,
                                        nullptr,
                                        nullptr,
                                        this,
                                        &lbfgs_params);


    double time_ms = (ros::Time::now() - t1).toSec() * 1000;

    jerkOpt_.getTraj(minJerkTraj_);
    int checker = isFeasibile();
    // std::cout << "Checker status is  " << checker << "result " << result  << std::endl;

    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGSERR_MAXIMUMITERATION || 
        result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
        result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
        result == lbfgs::LBFGS_STOP)
    {

      if (checker == bg_planner::CHECKER_TYPE::FEASIBLE)
      {
        printf("\033[32m Success: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
        return true;
      }else
      {

        printf("\033[33m Infeasible: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
      }

    }

    std::cout << "cannot solve it  " << std::endl;

    return false;
  }


  // return true if the trajectory is satisfying
  int PolySolver::isFeasibile()
  {
    // check the dynamic limits
    if (!minJerkTraj_.checkMaxVelRate(b_(0) + bb_(0)))
    {
      std::cout << " minJerkTraj_.getMaxVelRate() " << minJerkTraj_.getMaxVelRate() << std::endl;
      return bg_planner::CHECKER_TYPE::VEL_INFI;
    }
    if (!minJerkTraj_.checkMaxAccRate(b_(1) + bb_(1)))
    {
      std::cout << " minJerkTraj_.getMaxAccRate() " << minJerkTraj_.getMaxAccRate() << std::endl;
      return bg_planner::CHECKER_TYPE::ACC_INFI;
    }
    return bg_planner::CHECKER_TYPE::FEASIBLE;
  }

  template <typename EIGENVEC>
  void PolySolver::VT2RT(Eigen::VectorXd &RT, const EIGENVEC &VT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC>
  void PolySolver::RT2VT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  double PolySolver::objCallback(void *ptrObj,
                                 const double *x,
                                 double *grad,
                                 const int n)
  {
    PolySolver &obj = *(PolySolver *)ptrObj;

    // intialization
    //  map for x
    Eigen::Map<const Eigen::MatrixXd> innerP(x, 3, obj.piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> alloVT(x + (3 * (obj.piece_num_ - 1)), obj.piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, obj.piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradVT(grad + (3 * (obj.piece_num_ - 1)), obj.piece_num_);
    gradVT.setZero();
    gradP.setZero();

    // convert the time T(>0) to e^T (virtual time)
    Eigen::VectorXd alloRT(obj.piece_num_);
    Eigen::VectorXd gradRT(obj.piece_num_);
    gradRT.setZero();
    obj.VT2RT(alloRT, alloVT);
    // map for gradients
    double smooth_cost = 0, piece_cost = 0;

    obj.jerkOpt_.generate(innerP, alloRT);

    smooth_cost = obj.jerkOpt_.getObjective(); // only the j hat
    gradRT += obj.jerkOpt_.getGradT();
    gradP  += obj.jerkOpt_.getGradInnerP();

    obj.jerkOpt_.addPieceCostGrad(gradRT, gradP, piece_cost, obj.SFCs_);
    obj.jerkOpt_.VTGrad(alloRT, alloVT, gradRT, gradVT);
    obj.iter_num_ += 1;

    return smooth_cost + piece_cost;
  }


  bool PolyYawSolver::minJerkYawOpt(Eigen::MatrixXd &inner_pts, // (1, N - 1)
                                    Eigen::VectorXd &allo_ts,
                                    Eigen::MatrixXd &yawState,
                                    bool use_ref)
  {

    innerP_ = inner_pts;
    alloT_ = allo_ts;
    initS_ = yawState.row(0);
    finalS_ = yawState.row(1);

    piece_num_ = alloT_.size();
    var_num_ = piece_num_ - 1; // don't optimize time

    yawOpt_.reset(initS_, finalS_, piece_num_);
    yawOpt_.generate(innerP_, alloT_);
    yawOpt_.setRef(innerP_);
    yawOpt_.getTraj(minInitTraj_);

    /*** record time ***/
    ros::Time t1 = ros::Time::now();

    /*** initialize the optimizer ***/
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-2;

    /*** optimize ***/
    double init_vars[var_num_];
    memcpy(init_vars, innerP_.data(), innerP_.size() * sizeof(init_vars[0]));
    double final_cost = 0;
    iter_num_ = 0;

    int result = lbfgs::lbfgs_optimize(var_num_,
                                        init_vars,
                                        &final_cost,
                                        PolyYawSolver::objCallback,
                                        nullptr,
                                        nullptr,
                                        this,
                                        &lbfgs_params);

    double time_ms = (ros::Time::now() - t1).toSec() * 1000;
    yawOpt_.getTraj(minJerkYawTraj_);

    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGSERR_MAXIMUMITERATION || 
        result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
        result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
        result == lbfgs::LBFGS_STOP)
    {

      int checker = isFeasibile();
      std::cout << "Checker status is  " << checker << std::endl;

      if (checker == bg_planner::CHECKER_TYPE::FEASIBLE)
      {
        printf("\033[32m Success: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
        return true;
      }
      else
      {
        // insert new points
        printf("\033[33m  infeasible: iter=%d, time(ms)=%5.3f, cost=%5.3f\n\033[0m", iter_num_, time_ms, final_cost);
        return false;
        
      }
    }

    return false;
  }

  double PolyYawSolver::objCallback(void *ptrObj,
                                    const double *x,
                                    double *grad,
                                    const int n)
  {
    PolyYawSolver &obj = *(PolyYawSolver *)ptrObj;

    // intialization
    //  map for x
    Eigen::Map<const Eigen::MatrixXd> innerP(x, 1, obj.piece_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 1, obj.piece_num_ - 1);
    gradP.setZero();

    // map for gradients
    double smooth_cost = 0, piece_cost = 0;

    obj.yawOpt_.generate(innerP, obj.alloT_);
    smooth_cost = obj.yawOpt_.getObjective(); // only the j hat
    gradP += obj.yawOpt_.getGradInnerP();

    obj.yawOpt_.addPieceCostGrad(gradP, piece_cost);
    obj.iter_num_ += 1;

    return smooth_cost + piece_cost;
  }

  int PolyYawSolver::isFeasibile()
  {
    // check the dynamic limits
    if (!minJerkYawTraj_.checkMaxVelRate(b_(0) + bb_(0)))
    {
      std::cout << "[check the dynamic limits ] minJerkYawTraj_.getMaxVelRate() " << minJerkYawTraj_.getMaxVelRate() << std::endl;
      return bg_planner::CHECKER_TYPE::VEL_INFI;
    }
    if (!minJerkYawTraj_.checkMaxAccRate(b_(1) + bb_(1)))
    {
      std::cout << "[check the dynamic limits ] minJerkYawTraj_.getMaxAccRate() " << minJerkYawTraj_.getMaxAccRate() << std::endl;
      return bg_planner::CHECKER_TYPE::ACC_INFI;
    }

    return bg_planner::CHECKER_TYPE::FEASIBLE;
  }

} // namespace
