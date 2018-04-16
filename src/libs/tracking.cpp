#include "tracking.h"


namespace followlib
{

PeopleTracker::PeopleTracker(const ros::NodeHandle &n, const PTCallback &cb):
    p_rosnode(n), p_cb(cb) {
  p_ptSub = n.subscribe("spencer/perception/tracked_persons", 
    1000, &PeopleTracker::p_SubCb, this);
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

}