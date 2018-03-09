#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
/*
  All function and class declarations pertaining to kinect mapping goes here.
  All names declared in the namespace must be unique.
*/

using namespace std;
using geometry_msgs::Point32;

namespace followlib 
{
  // all code goes here

  void trans_p(Point32 &p_o, Eigen::Vector3f &p_trans);
  void trans_pc(vector<Point32> &points, vector<Point32> &points_translated);

  /*
    A trivial test function, remove when stuff is implemented here.
    Purpose is to test library linking into main executable.
  */
  void trivial_test();
}