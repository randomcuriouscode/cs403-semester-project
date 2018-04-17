#pragma once
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/

//turtlesim move2goal implementation
//Constants
const std::string TURTLEBOT_POSE_TOPIC = "/turtle1/pose";
const std::string TURTLEBOT_CMD_VEL_TOPIC = "/turtle1/cmd_vel";
const float DISTANCE_EPS = .01; //epsilon for distance
const float LIN_CONS = 1.5; // linear vel constant
const float ANG_CONS = 6; //angular vel constant

namespace followlib
{
  // all code goes here
  class PathFinding
  {
  public:
    PathFinding(ros::NodeHandle &_n, turtlesim::Pose _goal);
    void pose_SubCb(const turtlesim::Pose &msg);
    float get_linear_vel(float dist);
    float get_angular_vel();
    void driveGoal();
    void drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z);

  private:
    const ros::NodeHandle &n;
    turtlesim::Pose &goal;
    turtlesim::Pose curr_location;
    ros::Subscriber pose_sub;
    ros::Publisher cmd_vel_pub;
  };
}
