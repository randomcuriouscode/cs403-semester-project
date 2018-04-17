#include <gtest/gtest.h>
#include <tracking.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <pthread.h>
#include <chrono>

#define GTEST

using namespace std;
using Eigen::Vector3f;
using namespace followlib;

ros::NodeHandle *handle;
bool callback_called = false;

void PTrackCallback(Eigen::Vector2d pt)
{
  callback_called = true;
}

/**
* This test will fail if the spencer library is not currently tracking a person.
*/
TEST(TestSuite, test_PTrack)
{ 
  PTCallback cb = PTrackCallback;
  PeopleTracker pt(*handle, cb);

  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < start + chrono::seconds(10))
  {
    ros::spin();
  } 

  ASSERT_TRUE(callback_called);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  handle = &nh;
  return RUN_ALL_TESTS();
}