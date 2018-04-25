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
  while(1==1){
    geometry_msgs::Pose dest;
    //position acquired from tracking
    dest.position.x = 1;
    dest.position.y = 0;
    dest.position.z = 0;
    PeopleTracker pt(n, &PTrackCallback);
    PathFinding pf = PathFinding(n); //cobot will spin in circles (which makes sense because goal isn't updated periodically)
    pf.driveTo(dest);
    ros::spin();
  }

  return(0);
}
