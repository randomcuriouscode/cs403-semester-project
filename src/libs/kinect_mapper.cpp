#include "kinect_mapper.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h> // logging
#include <iostream> // testing purposes

using Eigen::Vector3f;
using geometry_msgs::Point32;

void followlib::KinectMapper::pReadRigidTransform(string path)
{
  YAML::Node config = YAML::LoadFile(path);

  auto rotation = config["camera_rigid_transform"]["rotation"];
  auto translation = config["camera_rigid_transform"]["translation"];

    for (size_t row = 0; row < 3; row++)
  {
    for (size_t col = 0; col < 3; col ++)
    {
      p_R(row,col) = rotation[row][col].as<float>();
    }
  }

  p_T.x() = translation[0].as<float>();
  p_T.y() = translation[1].as<float>();
  p_T.z() = translation[2].as<float>();
}

void followlib::KinectMapper::trans_p(const Point32 &p_o, Vector3f &p_trans)
{
  Vector3f P(p_o.x, p_o.y, p_o.z);

  Vector3f P_prime = (p_R * P) + p_T;

  p_trans.x() = P_prime.x();
  p_trans.y() = P_prime.y();
  p_trans.z() = P_prime.z();
}

void followlib::KinectMapper::trans_pc(vector<Point32> &points, vector<Vector3f> &points_translated)
{
  for (vector<Point32>::const_iterator it = points.begin(); it != points.end(); it++)
  {
    Vector3f p_trans;
    trans_p(*it, p_trans);
    points_translated.push_back(p_trans);
  }
}