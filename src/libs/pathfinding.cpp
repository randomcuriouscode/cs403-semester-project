#include "pathfinding.h"
#include "util.h"

followlib::PathFinding::PathFinding(ros::NodeHandle &_n):
  n(_n){
    robot_laser_sub = _n.subscribe(LASER_SCAN_TOPIC, 4, &PathFinding::robot_laser_cb, this);
    //cmd_vel_pub = _n.advertise<cobot_msgs::CobotDriveMsg>(CMD_VEL_TOPIC, 10, this);
    cmd_vel_pub = _n.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10, this);
}

float followlib::PathFinding::get_linear_vel(float dist){
  float vel = LIN_CONS*dist;
  if(std::abs(vel) > MAX_LIN_VEL){
    vel = MAX_LIN_VEL;
  }
  return vel;
}

float followlib::PathFinding::get_angular_vel(geometry_msgs::Pose dest){
  float vel = ANG_CONS*(atan2(dest.position.y, dest.position.x));
  //May need to check if desired velocity is even achievable given maximum acceleration
  if(std::abs(vel) > MAX_ANG_VEL){
    vel = MAX_ANG_VEL;
  }
  return vel;
}

void followlib::PathFinding::robot_laser_cb(const sensor_msgs::LaserScan& laser_scan) {
  float curr_angle = laser_scan.angle_min;
  std::vector<geometry_msgs::Point32> points;
  for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i ){
    geometry_msgs::Point32 P;
    float range = laser_scan.ranges[i];
    if(range > laser_scan.range_max || range < laser_scan.range_min){
      curr_angle += laser_scan.angle_increment;
      continue;
    }
    P.x = range * std::cos(curr_angle);
    P.y = range * std::sin(curr_angle);
    P.z = 0;
    Eigen::Vector3f v = Eigen::Vector3f(P.x, P.y, P.z);
    Eigen::Matrix3f R;
    R(0,0) = 1;
    R(0,1) = 0;
    R(0,2) = 0;
    R(1,0) = 0;
    R(1,1) = 1;
    R(1,2) = 0;
    R(2,0) = 0;
    R(2,1) = 0;
    R(2,2) = 1;
    Eigen::Vector3f t = Eigen::Vector3f(.145, 0, .23); //need to change this
    Eigen::Vector3f tv = R*v + t;
    geometry_msgs::Point32 result;
    result.x = tv.x();
    result.y = tv.y();
    result.z = tv.z();
    points.push_back(result);
    curr_angle += laser_scan.angle_increment;
  }
  obstacles = points;
}

std::pair<bool, float> followlib::PathFinding::detect_obstacle(float v, float w, float clearance){
  //Check laserscan & see if obstacles in desired path
  float shortest_fpl = nan("");
  for(std::vector<geometry_msgs::Point32>::iterator it = obstacles.begin(); it!=obstacles.end(); ++it){
    //compute fpl for each obstacle and v,w
    geometry_msgs::Point32 P = *it;
    float fpl = nan("");
    Eigen::Vector2f p (P.x, P.y);
    if(w == 0){
      if (fabs(p.y()) <= ROBOT_RADIUS && std::signbit(p.x()) == std::signbit(v)){
        fpl = p.x() - ROBOT_RADIUS;
      }
    } else {
      Eigen::Vector2f c (0,(float) v / w);
      float r = c.norm();
      float dist_point = fabs((c - p).norm() - r);
      if(dist_point < ROBOT_RADIUS){
        Eigen::Vector2f cp = p-c;
        Eigen::Vector2f co = -c;
        float cp_norm = cp.norm();
        float co_norm = co.norm();
        float pco = acos(cp.dot(co)/(cp_norm*co_norm));
        float pcl = atan(ROBOT_RADIUS / r);
        float lco = std::abs(pco - pcl);
        fpl = lco * r;
      }
    }
    //if nan fpl, then no obst
    if(!std::isnan(fpl)){
      if(std::isnan(shortest_fpl)){
	       shortest_fpl = fpl;
      } else if(fpl < shortest_fpl){
	       shortest_fpl = fpl;
      }
    }
  }
  //if shortest_fpl = nan, then there is no obstacles
  //if shortest_fpl is > some threshold, then no obstacle
  std::pair<bool, float> ret;
  ret.second = shortest_fpl;
  if(std::isnan(shortest_fpl)){
    ret.first = false;
  } else {
    ret.first = true;
  }
  return ret;
}

void followlib::PathFinding::driveTo(geometry_msgs::Pose dest){
  ros::Rate loop_rate(30);
  float dist = util::norm(-dest.position.x, -dest.position.y, 0);
  //Advance towards goal until epsilon distance away
  float best_lin_x;
  float best_ang_z;
  while(dist>DISTANCE_EPS){
    float lin_x = get_linear_vel(dist);
    float lin_y = 0;
    float lin_z = 0;
    float ang_x = 0;
    float ang_y = 0;
    float ang_z = get_angular_vel(dest);
    best_lin_x = lin_x;
    best_ang_z = ang_z;
    std::pair<bool, float> detect_result = detect_obstacle(lin_x, ang_z, MIN_CLEARANCE);
    if(detect_result.first){
      //Obstacle in path
      float best_fpl = detect_result.second;
      float delta_ang_vel = delta_theta * MAX_ANG_ACCEL;
      float delta_lin_vel = delta_theta * MAX_LIN_ACCEL;
      //rough estimate of permissable change in ang_vel
      for(int i = 0; i < 6; ++i){
        //bounded by +- delta_lin_vel or delta_ang_vel
        float test_lin_x = ((float)(i)*(2*delta_lin_vel/5))+(lin_x - delta_lin_vel);
        if(test_lin_x > MAX_LIN_VEL){
          test_lin_x = MAX_LIN_VEL;
        }
        for(int j = 0; j < 6; ++j){
          //ang_z constrained by how far we can actually accel within a time stamp
          float test_ang_z = ((float)(j)*(2*delta_ang_vel/5))+(ang_z - delta_ang_vel);
          if(test_ang_z > MAX_ANG_VEL){
            test_ang_z = MAX_ANG_VEL;
          }
          detect_result = detect_obstacle(test_lin_x, test_ang_z, best_fpl - .1); //allow some leeway
          //For now, pick direction with best FPL. May need to revisit this
          if(std::isnan(detect_result.second)){
            best_lin_x = test_lin_x;
            best_ang_z = test_ang_z;
            best_fpl = detect_result.second;
            break; //can't beat an obstacle free path
          } else if(detect_result.second > best_fpl){
            best_lin_x = test_lin_x;
            best_ang_z = test_ang_z;
            best_fpl = detect_result.second;
          }
        }
      }
    }
    //if obstacle, poll different directions (v,w) and get best
    drive(best_lin_x, lin_y, lin_z, ang_x, ang_y, best_ang_z);
    loop_rate.sleep();
    float dist = util::norm(-dest.position.x, -dest.position.y, 0);
  }
  drive(0,0,0,0,0,0); //stop
}
void followlib::PathFinding::drive(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z){
  /*cobot_msgs::CobotDriveMsg cm;
  cm.v = lin_x;
  cm.w = ang_z;*/
  geometry_msgs::Twist tm;
  geometry_msgs::Vector3 lin;
  lin.x = lin_x;
  lin.y = 0;
  lin.z = 0;
  geometry_msgs::Vector3 ang;
  ang.z = 0;
  ang.y = 0;
  ang.z = ang_z;
  tm.linear = lin;
  tm.angular = ang;
  cmd_vel_pub.publish(tm);
  //cmd_vel_pub.publish(cm);
  ros::spinOnce();
}
