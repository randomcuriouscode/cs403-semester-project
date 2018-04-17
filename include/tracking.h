#pragma once

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <functional>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <deque>
#include <boost/bind.hpp>

/*
  All function and class declarations pertaining to person, object
  tracking goes here.
  All names declared in the namespace must be unique.
*/

using PTCallback = std::function<void(Eigen::Vector2d)>;
const std::string PTRACK_TOPICNAME = "/spencer/perception/tracked_persons";

namespace followlib
{
  class PeopleTracker
  {
  public:
    /**
    * @brief Create a PeopleTracker with callback for tracked person coordinates
    * @param n: node handle of core app
    * @param cb: callback accepting vector2f points, bound using std::mem_fn()
    ***/
    PeopleTracker(ros::NodeHandle &n, const PTCallback &cb);

    PeopleTracker (const PeopleTracker&) = delete;
    PeopleTracker& operator= (const PeopleTracker&) = delete;
  private:
    void p_SubCb(const spencer_tracking_msgs::TrackedPersons::ConstPtr &msg);
  private:
    const ros::NodeHandle &p_rosnode;
    const PTCallback &p_cb;
    ros::Subscriber p_ptSub;
  };
}