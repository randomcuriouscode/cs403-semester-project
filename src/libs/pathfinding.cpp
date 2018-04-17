#include "pathfinding.h"
#include "util.h"

followlib::PathFinding::PathFinding(ros::NodeHandle &_n, turtlesim::Pose _goal):
  n(_n), goal(_goal){
    pose_sub = _n.subscribe(TURTLEBOT_POSE_TOPIC, 10, &PathFinding::pose_SubCb, this);
    cmd_vel_pub = _n.advertise<geometry_msgs::Twist>(TURTLEBOT_CMD_VEL_TOPIC, 10, this);
}

void followlib::PathFinding::pose_SubCb(const turtlesim::Pose &msg){
  curr_location.x = msg.x;
  curr_location.y = msg.y;
  curr_location.theta = msg.theta;
}

float followlib::PathFinding::get_linear_vel(float dist){
  return LIN_CONS*dist;
}

float followlib::PathFinding::get_angular_vel(){
  return ANG_CONS*(atan2(goal.y - curr_location.y, goal.x - curr_location.x) - curr_location.theta);
}

void followlib::PathFinding::driveGoal(){
  ros::Rate loop_rate(20);
  float dist = util::norm(curr_location.x-goal.x, curr_location.y-goal.y, 0);
  //Advance towards goal until epsilon distance away
  while(dist>DISTANCE_EPS){
    float lin_x = get_linear_vel(dist);
    float lin_y = 0;
    float lin_z = 0;
    float ang_x = 0;
    float ang_y = 0;
    float ang_z = get_angular_vel();
    drive(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z);
    loop_rate.sleep();
    dist = util::norm(curr_location.x-goal.x, curr_location.y-goal.y, 0);
  }
  drive(0,0,0,0,0,0); //stop
}
void followlib::PathFinding::drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = lin_x;
  cmd_vel.linear.y = lin_y;
  cmd_vel.linear.x = lin_z;
  cmd_vel.angular.x = ang_x;
  cmd_vel.angular.y = ang_y;
  cmd_vel.angular.z = ang_z;
  cmd_vel_pub.publish(cmd_vel);
  ros::spinOnce();
}
