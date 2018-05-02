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
static double dist_thresh = 1.;
static double theta_thresh = .05;

// Define service and callback functions

void PTrackCallback(PathFinding& pathfinder, Eigen::Vector2d pt)
{
  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
      "PTrackCallback: Got (" <<
      pt.x() << "," << pt.y() << ")");
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "followbot");
  ros::NodeHandle n;

  if (argc == 3)
  {
    dist_thresh = atof(argv[1]);
    theta_thresh = atof(argv[2]);
  }

#ifdef DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
      "Starting with dist_thresh: " << dist_thresh <<
      " theta_thresh: " << theta_thresh);

  cout << "Spinning..." << endl;


  // stuff goes here

  PathFinding pathfinder(n, dist_thresh, theta_thresh);
#ifdef DEBUG
while(1==1){
  Eigen::Vector2d ptTest (4, 0);
  pathfinder.moveGoal(ptTest);
}
#endif
  PTCallback ptCb = [pathfinder](Eigen::Vector2d pt)->void{
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME,
      "PTrackCallback: Got (" <<
      pt.x() << "," << pt.y() << ")");
    pathfinder.moveGoal(pt);
  };
  //PeopleTracker pt(n, ptCb);

  ros::spin();

  return(0);
}
