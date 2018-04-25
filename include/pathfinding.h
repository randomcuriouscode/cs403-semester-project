#pragma once
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include "cobot_msgs/CobotDriveMsg.h"
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/

//cobot move2goal implementation
//Constants
//const std::string POSE_TOPIC = "/turtle1/pose";
const std::string LASER_SCAN_TOPIC = "/Cobot/Laser";
//const std::string LASER_SCAN_TOPIC = "/COMPSCI403/LaserScan";
const std::string CMD_VEL_TOPIC = "/Cobot/Drive";
//const std::string CMD_VEL_TOPIC = "/cmd_vel_mux/input/navi";
const float DISTANCE_EPS = .01; //epsilon for distance
const float LIN_CONS = 1.5; // linear vel constant
const float ANG_CONS = 6; //angular vel constant
const float MAX_LIN_VEL = .25;
const float MAX_ANG_VEL = 1.5;
const float MAX_LIN_ACCEL = .5;
const float MAX_ANG_ACCEL = 2;
const float delta_theta = .0333;
const float ROBOT_RADIUS = .18;
const float MIN_CLEARANCE = 1.5;

namespace followlib
{
  // all code goes here
  class PathFinding
  {
  public:
    PathFinding(ros::NodeHandle &_n);
    float get_linear_vel(float dist);
    float get_angular_vel(geometry_msgs::Pose dest);
    std::pair<bool, float> detect_obstacle(float v, float w, float clearance);
    void driveTo(geometry_msgs::Pose dest);
    void drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z);

  private:
    void robot_laser_cb(const sensor_msgs::LaserScan& laser_scan);
  private:
    const ros::NodeHandle &n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber robot_laser_sub;
    std::vector<geometry_msgs::Point32> obstacles;
  };
}
