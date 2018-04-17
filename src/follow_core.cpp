#include "follow_core.h"
#include "kinect_mapper.h"
#include "pathfinding.h"
#include "tracking.h"
#include "util.h"
#include <ros/console.h>

using namespace std;
using namespace followlib;

#define DEBUG // comment out if not debug
const string LOGGER_NAME = "FollowCore";

// Define service and callback functions

void PTrackCallback(Eigen::Vector2d pt)
{
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, 
      "PTrackCallback: Got (" << 
      pt.x() << "," << pt.y() << ")");
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "followbot");
  ros::NodeHandle n;

#ifdef DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
  }
#endif

  cout << "Spinning..." << endl;

  // stuff goes here
  PeopleTracker pt(n, &PTrackCallback);

  ros::spin();

  return(0);
}