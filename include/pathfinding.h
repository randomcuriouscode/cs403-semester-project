#pragma once
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "cobot_msgs/CobotDriveMsg.h"
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/

//cobot move2goal implementation
//Constants
const std::string TURTLEBOT_POSE_TOPIC = "/turtle1/pose";
const std::string TURTLEBOT_CMD_VEL_TOPIC = "/Cobot/Drive";
const float DISTANCE_EPS = .01; //epsilon for distance
const float LIN_CONS = 1.5; // linear vel constant
const float ANG_CONS = 6; //angular vel constant
const float MAX_LIN_VEL = .5;
const float MAX_ANG_VEL = 1.5;
const float MAX_LIN_ACCEL = .5;
const float MAX_ANG_ACCEL = 2;
const float delta_theta = .0333;

namespace followlib
{
  // all code goes here
  class PathFinding
  {
  public:
    PathFinding(ros::NodeHandle &_n, geometry_msgs::Pose _goal);
    float get_linear_vel(float dist);
    float get_angular_vel();
    void driveGoal();
    void drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z);

  private:
    const ros::NodeHandle &n;
    geometry_msgs::Pose goal;
    ros::Publisher cmd_vel_pub;
  };
}
