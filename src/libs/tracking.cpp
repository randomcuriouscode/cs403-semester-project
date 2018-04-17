#include "tracking.h"
#include <ros/transport_hints.h>

using namespace followlib;

PeopleTracker::PeopleTracker(ros::NodeHandle &n, const PTCallback &cb):
    p_rosnode(n), p_cb(cb) {
  p_ptSub = n.subscribe(PTRACK_TOPICNAME, 1000, &PeopleTracker::p_SubCb, this);
}

void PeopleTracker::p_SubCb(const spencer_tracking_msgs::TrackedPersons::ConstPtr &msg)
{ 
  const spencer_tracking_msgs::TrackedPersons &m = *msg;
  if (m.tracks.size() > 0)
  {
    const geometry_msgs::Point &p = m.tracks[0].pose.pose.position;
    Eigen::Vector2f v (p.x, p.y);
    p_cb(v);
  }
}