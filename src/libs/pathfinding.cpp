#include "pathfinding.h"

const std::string LOGGER_NAME = "PathFinding";

#define DEBUG

#ifdef DEBUG
#include "cobot_msgs/CobotDriveMsg.h"
#else
#include <geometry_msgs/Twist.h>
#endif

followlib::PathFinding::PathFinding(double d, double t, ros::NodeHandle &n){
  nh = n;
  dist_thresh = d;
  theta_thresh = t;
#ifdef DEBUG
  cmd_vel_pub = nh.advertise<cobot_msgs::CobotDriveMsg>("/Cobot/Drive", 1);
#else
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
#endif
}

void followlib::PathFinding::moveGoal(Eigen::Vector2d goal) const{
  Eigen::Vector2d initLocation(0,0);
  Eigen::Vector2d oX(1,0);
  Eigen::Vector2d oG(goal.x(),goal.y());
  int turn = 1;
  if(goal.y() < 0){
    turn = -1;
  }
  double mul = oX.x()*oG.x() + oX.y()*oG.y();
  double angle = acos(mul/(oX.norm()*oG.norm()));
  //calculate linear velocity
  double v = 0.2;
  if(sqrt(goal.x()*goal.x() + goal.y()*goal.y()) <= dist_thresh)
    v = 0;

  double w;
  if(angle <= theta_thresh)
    w = 0;
  else
    w = W_MAX;

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, 
      "moveGoal: Publishing (" << 
      v << "," << w << ")");

#ifdef DEBUG 
  //calculate angular velocity
  cobot_msgs::CobotDriveMsg vw;

  vw.v = v;
  vw.w = w * turn;
  cmd_vel_pub.publish(vw);
#else
  geometry_msgs::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w * turn;

  cmd_vel_pub.publish(msg);
#endif
}
