#pragma once

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

#include "follow_core.h"
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/


namespace followlib
{
  // all code goes here
  class PathFinding{
  private:
    turtlesim::Pose initLocation;
    turtlesim::Pose finalLocation;
    double distance_tolerance;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber pose_sub;
    ros::NodeHandle nh;

  public:
    /*
      goal: final location to reach
      n: NodeHandle
    */
    PathFinding(turtlesim::Pose goal, double d, ros::NodeHandle &n);
    /*
      helper function fo get distance between 2 points
    */
    double getDistance(double x1, double y1, double x2,double y2);

    /*
      move to finalLocation
    */
    void moveGoal();

    /*
      get current location of the turtleBot
    */
    void poseCallBack(const turtlesim::Pose &p_msg);
  };
}
