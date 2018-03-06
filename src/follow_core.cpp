#include "follow_core.h"
#include "kinect_mapper.h"
#include "pathfinding.h"
#include "tracking.h"
#include "util.h"

using namespace std;

// Define service and callback functions

int main(int argc, char **argv) {

  ros::init(argc, argv, "followbot");
  ros::NodeHandle n;

  cout << "Hello World!" << endl;

  followlib::trivial_test();
  // stuff goes here

  ros::spin();

  return(0);
}