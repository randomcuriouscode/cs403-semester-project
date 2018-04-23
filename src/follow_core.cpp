#include "follow_core.h"
#include "kinect_mapper.h"
#include "pathfinding.h"
#include "tracking.h"
#include "util.h"
#include <ros/console.h>
#include "cobot_msgs/CobotDriveMsg.h"

using namespace std;
using namespace followlib;

#define DEBUG // comment out if not debug
const string LOGGER_NAME = "FollowCore";
ros::Publisher pub;
ros::Subscriber sub2;
float goalX = 10;
float goalY = 1.5;
float x = 0;
float y = 0;
float fv = 0;
float av = 0;

// Define service and callback functions
void getOdom(const nav_msgs::Odometry::ConstPtr& msg){
  x = goalX - msg->pose.pose.position.x;
  y = goalY - msg->pose.pose.position.y;

  fv = msg->twist.twist.linear.x;
  av = msg->twist.twist.angular.z;
}

void laserCallback(sensor_msgs::LaserScan msg){

  PathFinder pathFinder = PathFinder(x, y, msg, fv, av);
  cobot_msgs::CobotDriveMsg cb;
  cb.header = msg.header;
  cb.v = pathFinder.getForwardVelocity();
  cb.w = pathFinder.getAngularVelocity();
  pub.publish(cb);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "followbot");
  ros::NodeHandle n;
  pub = n.advertise<cobot_msgs::CobotDriveMsg>("/Cobot/Drive", 1000);
  sub2 = n.subscribe<nav_msgs::Odometry>("/odom", 1000, getOdom);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/Cobot/Laser", 1000, laserCallback);

  ros::spin();

  return(0);
}