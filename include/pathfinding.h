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

//Constants
const std::string LASER_SCAN_TOPIC = "/Cobot/Laser";
//const std::string LASER_SCAN_TOPIC = "/COMPSCI403/LaserScan"; //Kinect Mapper?
const std::string CMD_VEL_TOPIC = "/Cobot/Drive";
//const std::string CMD_VEL_TOPIC = "/cmd_vel_mux/input/navi";
const double DISTANCE_EPS = .9; //distance permissable from tracked person
const double LIN_CONS = 1.5; // linear vel constant
const double ANG_CONS = 6; //angular vel constant
const double MAX_LIN_VEL = .5;
const double MAX_ANG_VEL = 1.5;
const double MAX_LIN_ACCEL = .5;
const double MAX_ANG_ACCEL = 2;
const double delta_time = .0333;
const double ROBOT_RADIUS = .18;
const double MIN_CLEARANCE = 3;
//Cost function
const double ALPHA = -5;
const double BETA = -5;
const double GAMMA = -5;
namespace followlib
{
  // all code goes here
  class PathFinding
  {
  public:
    PathFinding(ros::NodeHandle &_n, double d, double t);
    double get_linear_vel(double dist);
    double get_angular_vel(Eigen::Vector2d dest);
    std::pair<bool, double> detect_obstacle(double v, double w, double clearance);
    void driveTo(Eigen::Vector2d dest);
    void drive(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z);
  private:
    void robot_laser_cb(const sensor_msgs::LaserScan& laser_scan);
  private:
    const ros::NodeHandle &n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber robot_laser_sub;
    std::vector<geometry_msgs::Point32> obstacles;
    double dist_thresh;
    double theta_thresh;
  };
}
