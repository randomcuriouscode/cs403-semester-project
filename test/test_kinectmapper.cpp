#include <gtest/gtest.h>
#include <kinect_mapper.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#define GTEST

using namespace std;
using Eigen::Vector3f;

TEST(TestSuite, test_RigidConfigParse)
{
  followlib::KinectMapper m("test/kinect_test.yaml");

  for (int row = 0; row < 3; row ++)
  {
    for (int col = 0; col < 3; col ++)
    {
      ASSERT_EQ(m.R()(row, col), row * 3 + col);
    }
  }

  ASSERT_EQ(m.T().x(), 0.f);
  ASSERT_EQ(m.T().y(), 1.f);
  ASSERT_EQ(m.T().z(), 2.f);
}

TEST(TestSuite, test_TranslatePoint)
{
  followlib::KinectMapper m("test/kinect_test.yaml");

  geometry_msgs::Point32 p;
  p.x = 1.0;
  p.y = 2.0;
  p.z = 3.0;

  Vector3f p_trans;

  m.trans_p(p, p_trans);

  ASSERT_NE(p_trans.x(), p.x);
  ASSERT_NE(p_trans.y(), p.y);
  ASSERT_NE(p_trans.z(), p.z);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}