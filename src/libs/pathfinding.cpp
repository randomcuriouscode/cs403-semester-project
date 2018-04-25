#include "pathfinding.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;
using geometry_msgs::Point32;

float followlib::PathFinder::distance(geometry_msgs::Point32 p, float v, float w){
	float dist = -1;
	Vector3f robot = Vector3f(0,0,0);
	Vector3f point = Vector3f(p.x, p.y, 0);
	if(point.norm() <= robotRadius){
		return 0;
	}
	if(w == 0.0){
		if(fabs(p.y) <= robotRadius){
			dist = point.x() - (sqrt((pow(robotRadius, 2)-pow(point.y(), 2))));
		}
	}
	else{
		float radius = v/w;
		if(radius == 0.0){
			return 0;
		}
		Vector3f centerRotation = Vector3f(0, radius, 0);
		float collisionY = -((centerRotation - point).norm() - fabs(radius));
		if(fabs(collisionY) < robotRadius){
			float collisionX = sqrt(fabs(pow(robotRadius, 2) - pow(collisionY, 2)));
			Vector3f robotCollision = -centerRotation + Vector3f(collisionX, collisionY, 0);
			Vector3f pointFromR = -centerRotation + point;
			float angleToPoint =  acos(pointFromR.dot(-centerRotation)/(pointFromR.norm()*centerRotation.norm()));
			float angleToCollision =  acos(robotCollision.dot(-centerRotation)/(robotCollision.norm()*centerRotation.norm()));
			float angle = angleToPoint - angleToCollision;
			dist = fabs(angle*radius);
		}
	}
	return dist;
}

float followlib::PathFinder::distanceToPoint(float fv, float av, float x, float y){
	float deltaX = 0;
	float deltaY = 0;
	if(av == 0){
		deltaX = fv*timeStep;
	}
	else{
		float radius = fv/av;
		float theta = av*timeStep;
		float chord = 2 * radius * std::sin(0.5 * theta);
		deltaX = chord * std::cos(theta);
		deltaY = chord * std::sin(theta);
	}
		float dist = (x - deltaX) * (x - deltaX) + (y - deltaY) * (y - deltaY);
		return dist;
}

float score(float v, float w, float dist, float goal, float tracking){
	float a = 1;		//forward veloctiy 
	float b = 1;		//angular
	float c = 10;		//free path
	float d = 10;		// distance to goal
	float s = a*v + b*abs(w) + c*dist;
	//float score = a*v + b*abs(w) + c*dist + 
	return s;
}

float score2(float v, float w, float dist, float obstacle){
	float a = 1;		//forward veloctiy 
	float b = 1;		//angular
	float c = 10;		//free path
	float d = 10;		// distance to obstacle
	float s = a*v + b*abs(w) + c*dist;
	return s;
}

void followlib::PathFinder::checkCurrPath(){
	geometry_msgs::Point32 point;
	Matrix3f r = Matrix3f::Identity();
	Vector3f t = Vector3f(0.145, 0, 0.23);
	Vector3f p;
	Vector3f rrf;
	float angle;
	obstacles.clear();
	//convert laser scan to Point[]
	for(int i = 0; i < laserScan.ranges.size(); i++){
		angle = (laserScan.angle_min + (laserScan.angle_increment * i));
		if(angle <=  laserScan.angle_max && laserScan.ranges[i]>= laserScan.range_min && laserScan.ranges[i] <= laserScan.range_max){
			point.x = laserScan.ranges[i]*std::cos(angle);
			point.y = laserScan.ranges[i]*std::sin(angle);
			p = Vector3f(point.x, point.y, point.z);
			rrf = r*p+t;
			point.x = rrf.x();
			point.y = rrf.y();
			obstacles.push_back(point);
		}
	}

	float maxForward = forwardV + maxAF * timeStep;
	float maxAngular = angularV + maxAA * timeStep;
	float minForward = forwardV - maxAF * timeStep;
	float minAngular = angularV - maxAA * timeStep;
	if(maxForward > maxForwardV){
		maxForward = maxForwardV;
	}
	if(minForward <0){
		minForward = 0;
	}
	if(maxAngular > maxAngularV){
		maxAngular = maxAngularV;
	}
	if(minAngular < -maxAngularV){
		minAngular = -maxAngularV;
	}
	float forwardVStep = (maxForward - minForward)/10;
	float angularVStep = (maxAngular - minAngular)/10;
	float tempDist = 0;
	float minDist = -1;
	float bestPath1 = -1;
	float bestPath2 = -1;
	float tempBestPath = 0;
	float bestFV = 0;
	float bestAV = 0;
	float index = 0;
	bool obstaclesInRange = true;
	float currScore = 0;
	float bestScore = -1;
	float bestScore2 = -1;
	//step through possible v and w
	for(float fv = minForward; fv <= maxForward; fv+=forwardVStep){
		for(float av = minAngular; av <= maxAngular; av+=angularVStep){
			minDist = -1;
			for(int i = 0; i<obstacles.size(); i++){
				tempDist = distance(obstacles[i], fv, av);
				if(tempDist != -1 && (minDist == -1 || tempDist < minDist)){
					minDist = tempDist;
					index = i;
				}
			}

			if(minDist == -1){
				if(av == 0){
					minDist = 4;
				}
				else{
					minDist = 2*3.14159265*fv/av / 4;
					if(minDist > 4){
						minDist = 4;
					}
				}
			}

			

			if(minDist > obstacleRange){
				obstaclesInRange = false;
				currScore = score(fv, av, minDist, distanceToPoint(fv, av, goalX, goalY),trackingRange);
				if(bestScore == -1 || currScore > bestScore){
					bestScore = currScore;
					bestFV = fv;
					bestAV = av;
				}
			}
			else if(obstaclesInRange){
				currScore = score2(fv, av, minDist, distanceToPoint(fv, av, obstacles[index].x, obstacles[index].y));
				if(bestScore2 == -1 || currScore > bestScore2){
					bestScore2 = currScore;
					bestFV = fv;
					bestAV = av;
				}
			}
		}
	}
	forwardV = bestFV;
	angularV = bestAV;

}

float followlib::PathFinder::getForwardVelocity(){
	return forwardV;
}

float followlib::PathFinder::getAngularVelocity(){
	return angularV;
}