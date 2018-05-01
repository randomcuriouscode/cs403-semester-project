#pragma once
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <nav_msgs/Odometry.h>
#define DEBUG

#ifdef DEBUG
#include "cobot_msgs/CobotDriveMsg.h"
#else
#include <geometry_msgs/Twist.h>
#endif
/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/
double W_MAX = 0.1;

//Constants
const std::string ODOMETRY_TOPIC = "/odom";
#ifdef DEBUG
const std::string LASER_SCAN_TOPIC = "/Cobot/Laser";
const std::string CMD_VEL_TOPIC = "/Cobot/Drive";
#else
const std::string LASER_SCAN_TOPIC = "/COMPSCI403/LaserScan";
const std::string CMD_VEL_TOPIC = "/cmd_vel_mux/input/navi";
#endif
const double DISTANCE_EPS = 2; //distance permissable from tracked person
const double LIN_CONS = 1.5; // linear vel constant
const double ANG_CONS = 6; //angular vel constant
const double MAX_LIN_VEL = .5;
const double MAX_ANG_VEL = .3;
const double MAX_LIN_ACCEL = .5;
const double MAX_ANG_ACCEL = 2;
const double delta_time = .0333;
const double ROBOT_RADIUS = .18;
//const double MIN_CLEARANCE = 3;
//Cost function
const double ALPHA = 5;
const double BETA = -5;
const double GAMMA = -5;
namespace followlib
{
  // all code goes here
  class PathFinding
  {
  public:
    PathFinding(ros::NodeHandle &_n, double d_thresh, double t_thresh);
    double get_linear_vel(double dist) const;
    double get_angular_vel(Eigen::Vector2d dest) const;
    std::pair<bool, double> detect_obstacle(double v, double w, double clearance) const;
    void moveGoal(Eigen::Vector2d dest) const;
    void drive(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z) const;
  private:
    void robot_laser_cb(const sensor_msgs::LaserScan& laser_scan);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  private:
    const ros::NodeHandle &n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber robot_laser_sub;
    ros::Subscriber odom_sub;
    std::vector<geometry_msgs::Point32> obstacles;
    double dist_thresh;
    double theta_thresh;
    double prev_ang_z;
    double prev_lin_x;
  };
}
