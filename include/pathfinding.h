#pragma once
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>

/*
  All function and class declarations pertaining to pathfinding goes here.
  All names declared in the namespace must be unique.
*/

namespace followlib 
{
  // all code goes here
	class PathFinder{
  		
		private: 
			float timeStep = 0.05;
			float robotRadius = 0.25;	//meters
			float maxForwardV = 0.5;	//m/s
			float maxAngularV = 1.5;	//rad/s	
			float maxAF = 0.5;
			float maxAA = 2.0;
			float obstacleRange = .5;	//range when we start to avoid obstacle
			float trackingRange = 2;		//follow distance
			float goalX;		
			float goalY;
			float forwardV;
			float angularV;
			sensor_msgs::LaserScan laserScan;
			std::vector<geometry_msgs::Point32> obstacles;
			float distance(geometry_msgs::Point32 p, float v, float w);
			float distanceToPoint(float fv, float av, float x, float y);	//calculates how close to goal new trajectory will get us
			void checkCurrPath();

		public:
			PathFinder(float x, float y, sensor_msgs::LaserScan ls, float currFV, float currAV){
				goalX = x;
				goalY = y;
				laserScan = ls;
				forwardV = currFV;
				angularV = currAV;
				checkCurrPath();
			}

			float getForwardVelocity();
			float getAngularVelocity();


	};
}