#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
/*
  All function and class declarations pertaining to kinect mapping goes here.
  All names declared in the namespace must be unique.
*/

using namespace std;

namespace followlib 
{
  // all code goes here
  class KinectMapper
  {
    
  private:
    void pReadRigidTransform(string path);
    Eigen::Matrix3f p_R;
    Eigen::Vector3f p_T;

  public:
    const Eigen::Matrix3f& R() { return p_R; }
    const Eigen::Vector3f& T() { return p_T; }

  public:

  /*
    @param configpath path to the configuration file relative
            to binary location
  */
  KinectMapper(string configpath)
  {
    pReadRigidTransform(configpath);
  }

  KinectMapper (const KinectMapper&) = delete;
  KinectMapper& operator= (const KinectMapper&) = delete;
  
  /*
    @brief Translate point (in kinect reference frame) to robot reference frame
    @param[in] p_o The input point
    @param[out] p_trans The output point
  */
  void trans_p(const geometry_msgs::Point32 &p_o, Eigen::Vector3f &p_trans);

  /*
    @brief Translate input points (in kinect reference frame) to robot reference frame
    @param[in] points The input points
    @param[out] points_translated The output points
  */
  void trans_pc(vector<geometry_msgs::Point32> &points, vector<Eigen::Vector3f> &points_translated);

  };
}