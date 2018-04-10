#include "pathfinding.h"

followlib::PathFinding::PathFinding(turtlesim::Pose goal, double d, ros::NodeHandle &n){
  finalLocation = goal;
  nh = n;
  distance_tolerance = d;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  pose_sub = n.subscribe("/turtle1/pose", 1000, &PathFinding::poseCallBack,this);

}
void followlib::PathFinding::poseCallBack(const turtlesim::Pose &p_msg){
  initLocation.x = p_msg.x;
  initLocation.y = p_msg.y;
  initLocation.theta = p_msg.theta;
}

double followlib::PathFinding::getDistance(double x1, double y1, double x2,double y2){
  return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

void followlib::PathFinding::moveGoal(){
  geometry_msgs::Twist vel_msg;

  ros::Rate loop_rate(10);

  do{
    vel_msg.linear.x = 1.5*getDistance(initLocation.x,initLocation.y,finalLocation.x,finalLocation.y);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 4*(atan2(finalLocation.y-initLocation.y,finalLocation.x-initLocation.x) - initLocation.theta);

    cmd_vel_pub.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  while(getDistance(initLocation.x,initLocation.y,finalLocation.x,finalLocation.y) > distance_tolerance);

  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;

  cmd_vel_pub.publish(vel_msg);

}
