#include <gtest/gtest.h>
#include <pathfinding.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#define GTEST

using namespace std;
using namespace followlib;

ros::NodeHandle *handle;

TEST(TestSuite, test_navi)
{
  Eigen::Vector2d goal (.5,.5);
  PathFinding p (1., .05, *handle); 

  ros::Rate r(30);
  while(ros::ok())
  {
    std::cerr << "moving.." << endl;
    p.moveGoal(goal);
    r.sleep();
    ros::spinOnce();
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  handle = &nh;
  return RUN_ALL_TESTS();
}