#include "tracking.h"
#include <ros/transport_hints.h>

using namespace followlib;

const std::string LOGGER_NAME = "PeopleTracker";

PeopleTracker::PeopleTracker(ros::NodeHandle &n, const PTCallback &cb):
    p_rosnode(n), p_cb(cb) {
  p_ptSub = n.subscribe(PTRACK_TOPICNAME, 4, &PeopleTracker::p_SubCb, this);
}

void PeopleTracker::p_SubCb(const spencer_tracking_msgs::TrackedPersons::ConstPtr &msg)
{ 
  const spencer_tracking_msgs::TrackedPersons &m = *msg;
  if (m.tracks.size() > 0)
  {
    const geometry_msgs::Point &p = m.tracks[0].pose.pose.position;
    Eigen::Vector2d v (p.x, p.y);
    p_lastReading = Eigen::Vector2d(v);
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, 
      "PeopleTracker::p_SubCb: Calling cb (" << 
      v.x() << "," << v.y() << ")");
    p_cb(v);
  }
  else if (p_lastReading.x() != 0. && p_lastReading.y() != 0.)
  {
    ROS_DEBUG_STREAM_NAMED(LOGGER_NAME, 
      "PeopleTracker::p_SubCb: Calling cb (" << 
      p_lastReading.x() << "," << p_lastReading.y() << ") (Default to previous track)");
    p_cb(p_lastReading);
  }
}