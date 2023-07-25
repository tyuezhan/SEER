/**
 * @file ellipsoid_decomp.h
 * @brief EllipsoidDecomp Class
 */
#ifndef ELLIPSOID_DECOMP_H
#define ELLIPSOID_DECOMP_H

#include <memory>
#include <decomp_util/line_segment.h>
#include <Eigen/Eigen>


/**
 * @brief EllipsoidDecomp Class
 *
 * EllipsoidDecomp takes input as a given path and find the Safe Flight Corridor around it using Ellipsoids
 */
template <int Dim>
class EllipsoidDecomp {
public:
 ///Simple constructor
 EllipsoidDecomp() {}
 /**
  * @brief Basic constructor
  * @param origin The origin of the global bounding box
  * @param dim The dimension of the global bounding box
  */
 EllipsoidDecomp(const Vecf<Dim> &origin, const Vecf<Dim> &dim) {
   global_bbox_min_ = origin;
   global_bbox_max_ = origin + dim;
 }

 ///Set obstacle points
 void set_obs(const vec_Vecf<Dim> &obs) { obs_ = obs; }

  ///Set dimension of bounding box 
  void set_local_bbox(const Vecf<Dim>& bbox, const Vecf<Dim>& lbox) {
  local_bbox_ = bbox;
  local_left_bbox_ = lbox;
  }

  void set_global_bbox(const Vecf<Dim> &origin, const Vecf<Dim> &dim) {
    global_bbox_min_ = origin;
    global_bbox_max_ = origin + dim;
   }
   

 ///Get the path that is used for dilation
 vec_Vecf<Dim> get_path() const { return path_; }

 ///Get the Safe Flight Corridor
 vec_E<Polyhedron<Dim>> get_polyhedrons() const { return polyhedrons_; }

 ///Get the ellipsoids
 vec_E<Ellipsoid<Dim>> get_ellipsoids() const { return ellipsoids_; }

 std::vector<Eigen::MatrixXd> get_hPoly(){
   std::vector<Eigen::MatrixXd>  hPolys;
   Eigen::MatrixXd hPoly;
   for(auto& it: polyhedrons_){
     auto vs = it.vs_;
     hPoly.resize(6, vs.size());
    
     //std::cout << "======================== " << std::endl;
     for (unsigned int i = 0; i < vs.size(); i++){
      auto v = vs[i];
      //std::cout << "v.p_ is " << v.p_ << std::endl;
      //std::cout << "v.n_ is " << v.n_ << std::endl;
      hPoly.col(i).head(3) = v.p_;
      hPoly.col(i).tail(3) = v.n_;   // normal must go outside

     }
     hPolys.push_back(hPoly);

   }
  
  return hPolys;
 }
  


 ///Get the constraints of SFC as \f$Ax\leq b \f$
 vec_E<LinearConstraint<Dim>> get_constraints() const {
   vec_E<LinearConstraint<Dim>> constraints;
   constraints.resize(polyhedrons_.size());
   for (unsigned int i = 0; i < polyhedrons_.size(); i++){
     const Vecf<Dim> pt = (path_[i] + path_[i+1])/2;
     constraints[i] = LinearConstraint<Dim>(pt, polyhedrons_[i].hyperplanes());
   }
   return constraints;
 }

 /**
  * @brief Decomposition thread
  * @param path The path to dilate
  * @param offset_x offset added to the long semi-axis, default is 0
  */
 void dilate(const vec_Vecf<Dim> &path, double offset_x = 0) {
   const unsigned int N = path.size() - 1;
   lines_.resize(N);
   ellipsoids_.resize(N);
   polyhedrons_.resize(N);

   for (unsigned int i = 0; i < N; i++) {
     lines_[i] = std::make_shared<LineSegment<Dim>>(path[i], path[i+1]);
     lines_[i]->set_local_bbox(local_bbox_, local_left_bbox_);
     lines_[i]->set_obs(obs_);
     lines_[i]->dilate(offset_x);

     ellipsoids_[i] = lines_[i]->get_ellipsoid();
     polyhedrons_[i] = lines_[i]->get_polyhedron();
   }


   path_ = path;

   if(global_bbox_min_.norm() != 0 || global_bbox_max_.norm() != 0) {
     for(auto& it: polyhedrons_)
       add_global_bbox(it);
   }

 }

protected:
 template<int U = Dim>
   typename std::enable_if<U == 2>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** add bound along X, Y axis

     //*** X
     Vs.add(Hyperplane2D(Vec2f(global_bbox_max_(0), 0), Vec2f(1, 0)));
     Vs.add(Hyperplane2D(Vec2f(global_bbox_min_(0), 0), Vec2f(-1, 0)));
     //*** Y
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_max_(1)), Vec2f(0, 1)));
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_min_(1)), Vec2f(0, -1)));
   }

 template<int U = Dim>
   typename std::enable_if<U == 3>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** add bound along X, Y, Z axis
     //*** Z
     Vs.add(Hyperplane3D(Vec3f(0.0, 0.0, global_bbox_max_(2)), Vec3f(0.0, 0.0, 1.0)));
     Vs.add(Hyperplane3D(Vec3f(0.0, 0.0, global_bbox_min_(2)), Vec3f(0.0, 0.0, -1.0)));

     //*** X
     Vs.add(Hyperplane3D(Vec3f(global_bbox_max_(0), 0.0, 0.0), Vec3f(1.0, 0.0, 0.0)));
     Vs.add(Hyperplane3D(Vec3f(global_bbox_min_(0), 0.0, 0.0), Vec3f(-1.0, 0.0, 0.0)));
     //*** Y
     Vs.add(Hyperplane3D(Vec3f(0.0, global_bbox_max_(1), 0.0), Vec3f(0.0, 1.0, 0.0)));
     Vs.add(Hyperplane3D(Vec3f(0.0, global_bbox_min_(1), 0.0), Vec3f(0.0, -1.0, 0.0)));  //bug issue
   }

 vec_Vecf<Dim> path_;
 vec_Vecf<Dim> obs_;

 vec_E<Ellipsoid<Dim>> ellipsoids_;
 vec_E<Polyhedron<Dim>> polyhedrons_;
 std::vector<std::shared_ptr<LineSegment<Dim>>> lines_;

 Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()};
 Vecf<Dim> local_left_bbox_{Vecf<Dim>::Zero()};

 Vecf<Dim> global_bbox_min_{Vecf<Dim>::Zero()}; // bounding box params
 Vecf<Dim> global_bbox_max_{Vecf<Dim>::Zero()};

};

typedef EllipsoidDecomp<2> EllipsoidDecomp2D;

typedef EllipsoidDecomp<3> EllipsoidDecomp3D;
#endif
