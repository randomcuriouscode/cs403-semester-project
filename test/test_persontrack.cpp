#include <gtest/gtest.h>
#include <tracking.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#define GTEST

using namespace std;
using Eigen::Vector3f;
using namespace followlib;

ros::NodeHandle *handle;

void PTrackCallback(Eigen::Vector2f pt)
{

}

TEST(TestSuite, test_PTrack)
{ 
  PTCallback cb = PTrackCallback;
  PeopleTracker pt(*handle, cb);
}

TEST(TestSuite, test_TranslatePoint)
{
  
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  handle = &nh;
  return RUN_ALL_TESTS();
}