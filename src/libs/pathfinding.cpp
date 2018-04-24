#include "pathfinding.h"
#include "util.h"

followlib::PathFinding::PathFinding(ros::NodeHandle &_n, geometry_msgs::Pose _goal):
  n(_n){
    goal = _goal;
    cmd_vel_pub = _n.advertise<cobot_msgs::CobotDriveMsg>(TURTLEBOT_CMD_VEL_TOPIC, 10, this);
}

float followlib::PathFinding::get_linear_vel(float dist){
  float vel = LIN_CONS*dist;
  if(std::abs(vel) > MAX_LIN_VEL){
    vel = MAX_LIN_VEL;
  }
  return vel;
}

float followlib::PathFinding::get_angular_vel(){
  float vel = ANG_CONS*(atan2(goal.position.y, goal.position.x));
  //May need to check if desired velocity is even achievable given maximum acceleration
  //... might not matter
  if(std::abs(vel) > MAX_ANG_VEL){
    vel = MAX_ANG_VEL;
  }
  return vel;
}

void followlib::PathFinding::driveGoal(){
  ros::Rate loop_rate(30);
  float dist = util::norm(-goal.position.x, -goal.position.y, 0);
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
    float dist = util::norm(-goal.position.x, -goal.position.y, 0);
  }
  drive(0,0,0,0,0,0); //stop
}
void followlib::PathFinding::drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
  /*geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = lin_x;
  cmd_vel.linear.y = lin_y;
  cmd_vel.linear.x = lin_z;
  cmd_vel.angular.x = ang_x;
  cmd_vel.angular.y = ang_y;
  cmd_vel.angular.z = ang_z;*/
  cobot_msgs::CobotDriveMsg cm;
  cm.v = lin_x;
  cm.w = ang_z;
  cmd_vel_pub.publish(cm);
  //cmd_vel_pub.publish(cmd_vel);
  ros::spinOnce();
}
