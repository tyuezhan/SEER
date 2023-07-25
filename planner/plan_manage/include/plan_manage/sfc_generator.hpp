#ifndef _SFC_GENERATOR_H_
#define _SFC_GENERATOR_H_

#include <stdlib.h>
#include <cmath>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>

/*** polytopes utils  ***/
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
/*** polytopes utils  ***/
#include "sdlp.hpp"

namespace sfc_gen
{

  class SfcGenerator
  {
    // SECTION stable
  public:
    SfcGenerator(){}
    ~SfcGenerator(){}

    /* main planning interface */
    void initROS(ros::NodeHandle &nh)
    {
      /* read algorithm parameters */
      nh.param("plan_frame_id", frame_id_, std::string("world"));
      // sfc initialization
      Eigen::Vector3d map_origin, map_size;
      nh.param("sdf_map/map_size_x", map_size(0), -1.0);
      nh.param("sdf_map/map_size_y", map_size(1), -1.0);
      nh.param("sdf_map/map_size_z", map_size(2), -1.0);
      nh.param("sdf_map/map_origin_x", map_origin(0), -1.0);
      nh.param("sdf_map/map_origin_y", map_origin(1), -1.0);
      nh.param("sdf_map/map_origin_z", map_origin(2), -1.0);
      rils_poly_gen_.set_global_bbox(Vec3f(map_origin(0), map_origin(1), map_origin(2)),
                                    Vec3f(map_size(0), map_size(1), map_size(2)));  
      rils_poly_gen_.set_local_bbox(Vec3f(2.0, 2.0, 1.0), Vec3f(2.0, 2.0, 1.0));
      rils_poly_pub_   =  nh.advertise<decomp_ros_msgs::PolyhedronArray>("rils_polys", 1, true);

    }
   
  private:

    // ROS related
    std::string frame_id_;
    ros::Publisher rils_poly_pub_;

    //RILS 
    EllipsoidDecomp3D rils_poly_gen_;
    double rils_bb_back_, rils_bb_front_;
    std::vector<Eigen::MatrixXd> hPolys_;

    inline bool checkInsidePoly(Eigen::Vector3d &pt,
                         Eigen::MatrixXd &hPoly)
    {
      Eigen::Vector3d p_, n_;
      for (int i = 0; i < hPoly.cols(); i++){
          p_ = hPoly.col(i).head<3>();
          n_ = hPoly.col(i).tail<3>();
        
          if ( n_.dot(pt- p_) > 1e-3){return false;}
      }
      return true;
    }

    //poly is defined as h0*x + h1*y + h2*z + h3 <= 0 
    inline bool overlap(const Eigen::MatrixXd &hPoly0,
                        const Eigen::MatrixXd &hPoly1,
                        const double eps = 0.001)

    {

        unsigned int m = hPoly0.cols();
        unsigned int n = hPoly1.cols();

        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);

        Eigen::MatrixX3d normals0 = (hPoly0.bottomRows<3>()).transpose();  //  (m, 3)
        Eigen::MatrixX3d normals1 = (hPoly1.bottomRows<3>()).transpose();  //  (n, 3)

        A.leftCols<3>().topRows(m) = normals0; // m * 3
        A.leftCols<3>().bottomRows(n) = normals1;
        A.rightCols<1>().setConstant(1.0);
        for (int i = 0; i < m; i ++)
        {
           b(i) =  normals0.row(i).dot(hPoly0.col(i).head<3>());
        }
        for (int j = 0; j < n; j ++)
        {
           b(m+j) =  normals1.row(j).dot(hPoly1.col(j).head<3>());
        }
        c.setZero();
        c(3) = -1.0;
        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }


  public:
    typedef std::unique_ptr<SfcGenerator> Ptr;

    inline void getPolyConst(std::vector<Eigen::Vector3d> &path_pts,
                              double path_time_res,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr,
                              Eigen::MatrixXd &ref_points,
                              Eigen::VectorXd &ref_ts,
                              std::vector<Eigen::MatrixXd> &hPolys) 
    {

      hPolys.clear();
      Eigen::MatrixXd hPoly;
      std::vector<double> temp_ts;
      std::vector<Eigen::MatrixXd> temp_hPolys;

      //setup the pointcloud
      vec_Vec3f vec_obs;
      vec_obs.resize(cloud_ptr->points.size());
      for (unsigned int i = 0; i < cloud_ptr->points.size(); i++) {
        vec_obs[i](0) = cloud_ptr->points[i].x;
        vec_obs[i](1) = cloud_ptr->points[i].y;
        vec_obs[i](2) = cloud_ptr->points[i].z;
      }
      rils_poly_gen_.set_obs(vec_obs);
      size_t path_size = path_pts.size();
      // step 1 : set the intial lengh
      // add start time
      temp_ts.push_back(0.0);
      int cnt_num = 0;
      int start_idx = 0, end_idx = 1;
      hPoly = getRILSConst(path_pts[start_idx], path_pts[end_idx]);

      for (size_t i = 0; i <= path_size-1; i++)
      {
        // check wehter or not we need to generate the point
        if (i > 0)
        {
          if (checkInsidePoly(path_pts[i], hPoly) && cnt_num <= 15)
          {
            cnt_num ++;
            continue;
          }
          else
          {
            end_idx = i-1;
            hPoly = getRILSConst(path_pts[start_idx], path_pts[end_idx]);
            temp_hPolys.push_back(hPoly);
            temp_ts.push_back(end_idx * path_time_res);

            cnt_num = 0;
            start_idx = end_idx;

          }
          
        }
      }
      
      /***Corridor cut-off and refinements***/
      //2. delete the overlap corridors
      int M = temp_hPolys.size();
      bool is_overlap;
      std::deque<int> idices;
      idices.push_front(M - 1);
      for (int i = M - 1; i >= 0; i--)
      {
        for (int j = 0; j < i; j++)
        {
          if (j < i - 1)
          {
            is_overlap = overlap(temp_hPolys[i], temp_hPolys[j], 0.5);
          }
          else
          {
            is_overlap = true;
          }
          if (is_overlap)
          {
            idices.push_front(j);
            i = j + 1;
            break;
          }
        }
      }
      // 3. set up the output
      int short_cut_size = idices.size()-1;

      if (short_cut_size  > 0)
      {

        ref_points.resize(3, short_cut_size);
        ref_ts.setZero(short_cut_size+1);
        int j = 0;
        int last_ele = 0;
        for (const auto &ele : idices)
        {
          hPolys.push_back(temp_hPolys[ele]);
          ref_ts(j) = temp_ts.at(ele+1) - temp_ts.at(last_ele);
          last_ele = ele+1;
          j +=1;
        }


        for (int i = 0; i < hPolys.size()-1; i ++)
        { 

          auto hPoly0 = hPolys[i];
          auto hPoly1 = hPolys[i+1];

          unsigned int m = hPoly0.cols();
          unsigned int n = hPoly1.cols();

          Eigen::MatrixX4d A(m + n, 4);
          Eigen::Vector4d c, x;
          Eigen::VectorXd b(m + n);

          Eigen::MatrixX3d normals0 = (hPoly0.bottomRows<3>()).transpose();  //  (m, 3)
          Eigen::MatrixX3d normals1 = (hPoly1.bottomRows<3>()).transpose();  //  (n, 3)

          A.leftCols<3>().topRows(m) = normals0; // m * 3
          A.leftCols<3>().bottomRows(n) = normals1;
          A.rightCols<1>().setConstant(1.0);

          for (int i = 0; i < m; i ++)
          {
            b(i) =  normals0.row(i).dot(hPoly0.col(i).head<3>());
          }
          for (int j = 0; j < n; j ++)
          {
            b(m+j) =  normals1.row(j).dot(hPoly1.col(j).head<3>());
          }

          c.setZero();
          c(3) = -1.0;

          const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
          
          ref_points.col(i) = x.head(3);

        }

      }else
      {
        ref_points.resize(3, 1);
        ref_ts.setZero(2);

        hPolys.push_back(hPoly);
        hPolys.push_back(hPoly);

        int temp_index = path_size/2;
        ref_ts(0) = temp_index* path_time_res;
        ref_ts(1) = temp_index* path_time_res;

        ref_points.col(0) =  0.5 * (path_pts[0] + path_pts[path_size-1]);
      }

      hPolys_ = hPolys;
      return;
    }


    inline Eigen::MatrixXd getRILSConst(Eigen::Vector3d pt1, 
                                              Eigen::Vector3d pt2)
    {
      Eigen::MatrixXd hPoly;
      Vec3f seed_point1, seed_point2;
      vec_Vec3f seed_path;
      seed_point1 << pt1(0), pt1(1), pt1(2);
      seed_point2 << pt2(0), pt2(1), pt2(2);
      seed_path.push_back(seed_point1);
      seed_path.push_back(seed_point2);
      rils_poly_gen_.dilate(seed_path);
      hPoly = rils_poly_gen_.get_hPoly()[0];
      return hPoly;
    }

    inline void visPoly()
    {
      decomp_ros_msgs::PolyhedronArray poly_msg;
      for (const auto &hPoly : hPolys_)
      {
        decomp_ros_msgs::Polyhedron msg;
        geometry_msgs::Point pt, n;
        for (unsigned int i = 0; i < hPoly.cols(); i++)
        {  
          pt.x = hPoly(0, i);
          pt.y = hPoly(1, i);
          pt.z = hPoly(2, i);
          n.x  = hPoly(3, i);
          n.y  = hPoly(4, i);
          n.z  = hPoly(5, i);
          msg.points.push_back(pt);
          msg.normals.push_back(n);
        }
        poly_msg.polyhedrons.push_back(msg);
      }
      poly_msg.header.frame_id = frame_id_;
      rils_poly_pub_.publish(poly_msg);
      return;
    }

    // !SECTION
  };
} // namespace

#endif