#include "pathfinding.h"

followlib::PathFinding::PathFinding(double d, double t, ros::NodeHandle &n){
  nh = n;
  dist_thresh = d;
  theta_thresh = t;
  cmd_vel_pub = nh.advertise<cobot_msgs::CobotDriveMsg>("/Cobot/Drive", 1);
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

  //calculate angular velocity
  cobot_msgs::CobotDriveMsg vw;
  double w;
  if(angle <= theta_thresh)
    w = 0;
  else
    w = W_MAX;

  vw.v = v;
  vw.w = w * turn;
  cmd_vel_pub.publish(vw);
}
