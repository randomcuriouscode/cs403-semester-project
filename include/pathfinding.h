#pragma once

#include <math.h>
#include <eigen3/Eigen/Dense>

#include "cobot_msgs/CobotDriveMsg.h"
#include "follow_core.h"
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/
double W_MAX = 0.1;

namespace followlib
{
  // all code goes here
  class PathFinding{
  private:
    double dist_thresh;
    double theta_thresh;
    ros::Publisher cmd_vel_pub;
    ros::NodeHandle nh;

  public:
    /*
      goal: final location to reach
      n: NodeHandle
    */
    PathFinding(double d, double t, ros::NodeHandle &n);

    /*
      move to finalLocation
    */
    void moveGoal(Eigen::Vector2d goal) const;

  };
}
