#include "pathfinding.h"
#include "util.h"

followlib::PathFinding::PathFinding(ros::NodeHandle &_n, double d, double t):
  n(_n){
    dist_thresh = d;
    theta_thresh = t;
    robot_laser_sub = _n.subscribe(LASER_SCAN_TOPIC, 4, &PathFinding::robot_laser_cb, this);
    cmd_vel_pub = _n.advertise<cobot_msgs::CobotDriveMsg>(CMD_VEL_TOPIC, 10, this);
    //cmd_vel_pub = _n.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10, this);
}

double followlib::PathFinding::get_linear_vel(double dist){
  double vel = LIN_CONS*dist;
  if(std::abs(vel) > MAX_LIN_VEL){
    vel = MAX_LIN_VEL;
  }
  return vel;
}

double followlib::PathFinding::get_angular_vel(Eigen::Vector2d dest){
  double vel = ANG_CONS*(atan2(dest.y(), dest.x()));
  //May need to check if desired velocity is even achievable given maximum acceleration
  if(std::abs(vel) > MAX_ANG_VEL){
    vel = MAX_ANG_VEL;
  }
  return vel;
}

void followlib::PathFinding::robot_laser_cb(const sensor_msgs::LaserScan& laser_scan) {
  double curr_angle = laser_scan.angle_min;
  std::vector<geometry_msgs::Point32> points;
  for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i ){
    geometry_msgs::Point32 P;
    double range = laser_scan.ranges[i];
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

std::pair<bool, double> followlib::PathFinding::detect_obstacle(double v, double w, double clearance){
  //Check laserscan & see if obstacles in desired path
  double shortest_fpl = nan("");
  for(std::vector<geometry_msgs::Point32>::iterator it = obstacles.begin(); it!=obstacles.end(); ++it){
    //compute fpl for each obstacle and v,w
    geometry_msgs::Point32 P = *it;
    double fpl = nan("");
    Eigen::Vector2f p (P.x, P.y);
    if(w == 0){
      if (fabs(p.y()) <= ROBOT_RADIUS && std::signbit(p.x()) == std::signbit(v)){
        fpl = p.x() - ROBOT_RADIUS;
      }
    } else {
      Eigen::Vector2f c (0,(double) v / w);
      double r = c.norm();
      double dist_point = fabs((c - p).norm() - r);
      if(dist_point < ROBOT_RADIUS){
        Eigen::Vector2f cp = p-c;
        Eigen::Vector2f co = -c;
        double cp_norm = cp.norm();
        double co_norm = co.norm();
        double pco = acos(cp.dot(co)/(cp_norm*co_norm));
        double pcl = atan(ROBOT_RADIUS / r);
        double lco = std::abs(pco - pcl);
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
  std::pair<bool, double> ret;
  ret.second = shortest_fpl;
  //While moving towards goal, if obstacle appears & is < 1m (?) then record as obstacle. Everything else will be treated
  //as not obstacles.
  if(std::isnan(shortest_fpl)){
    ret.first = false;
  } else if(shortest_fpl <= clearance){
    ret.first = true;
  } else {
    //There are obstacles, but outside of clearance
    ret.first = false;
  }
  return ret;
}

void followlib::PathFinding::driveTo(Eigen::Vector2d dest){
  ros::Rate loop_rate(30);
  double dist = util::norm(dest.x(), dest.y(), 0);
  //Advance towards goal until epsilon distance away
  double best_lin_vel;
  double best_ang_vel;
  double prev_lin_vel = 0;
  double prev_ang_vel = 0;
  while(dist>DISTANCE_EPS){
    Eigen::Vector2d initLocation(0,0);
    Eigen::Vector2d oX(1,0);
    Eigen::Vector2d oG(dest.x(),dest.y());
    int turn = 1;
    if(dest.y() < 0){
      turn = -1;
    }
    double mul = oX.x()*oG.x() + oX.y()*oG.y();
    double angle = acos(mul/(oX.norm()*oG.norm()));
    //calculate linear velocity
    double lin_vel = 0.2;
    double ang_vel;
    if(angle <= theta_thresh)
      ang_vel = 0;
    else
      ang_vel = MAX_ANG_VEL;
    /* Previous code for determining lin/ang vels
    double lin_x = get_linear_vel(dist);
    double ang_z = get_angular_vel(dest);
    best_lin_x = lin_x;
    best_ang_z = ang_z;
    */
    std::pair<bool, double> detect_result = detect_obstacle(lin_vel, ang_vel, MIN_CLEARANCE);
    if(detect_result.first){
      //Obstacle in path. Poll different directions to find a better one
      double best_cost = nan("");
      double best_fpl = detect_result.second;
      double delta_ang_vel = delta_time * MAX_ANG_ACCEL;//v=v0+at
      double delta_lin_vel = delta_time * MAX_LIN_ACCEL;
      int iterations = 10;
      //rough estimate of permissable change in ang_vel
      bool found_alt_direction = false;
      for(int i = 0; i < iterations+1; ++i){
        //bounded by +- delta_lin_vel or delta_ang_vel
        double test_lin_vel = ((double)(i)*(2*delta_lin_vel/iterations))+(lin_vel - delta_lin_vel);
        if(test_lin_vel > MAX_LIN_VEL){
          test_lin_vel = MAX_LIN_VEL;
        } else if (test_lin_vel < MAX_LIN_VEL*-1){
          test_lin_vel = MAX_LIN_VEL*-1;
        }
        for(int j = 0; j < 11; ++j){
          //ang_vel constrained by how far we can actually accel within a time stamp
          double test_ang_vel = ((double)(j)*(2*delta_ang_vel/iterations))+(ang_vel - delta_ang_vel);
          if(test_ang_vel > MAX_ANG_VEL){
            test_ang_vel = MAX_ANG_VEL;
          } else if (test_ang_vel < MAX_ANG_VEL*-1){
            test_ang_vel = MAX_ANG_VEL*-1;
          }
          detect_result = detect_obstacle(test_lin_vel, test_ang_vel, MIN_CLEARANCE); // maybe
          //compute stopping distance
          double vCriticalSq = nan("");
          if(!std::isnan(detect_result.second)) {
            vCriticalSq = (double)(2*MAX_LIN_ACCEL*detect_result.second);
          }
          double deltaTheta = std::abs(prev_ang_vel * delta_time);
          double wCriticalSq = (double)(2*MAX_ANG_ACCEL*deltaTheta);
          if((std::isnan(detect_result.second)) || (((prev_lin_vel*prev_lin_vel) < vCriticalSq) && ((prev_ang_vel*prev_ang_vel) < wCriticalSq))){
            //admissable, compute cost function
            //For now, pick direction with best FPL. Replace this with a scoring function.
            if(std::isnan(detect_result.second)){
              detect_result.second = 10; //nan fpl means no obstacles. Assign it a high value for purposes of cost function
            }
            double cost = BETA*detect_result.second + GAMMA*test_lin_vel;
            if(std::isnan(best_cost)){
              best_lin_vel = test_lin_vel;
              best_ang_vel = test_ang_vel;
              best_fpl = detect_result.second;
              best_cost = cost;
              found_alt_direction = true;
            } else if(cost < best_cost){
              best_lin_vel = test_lin_vel;
              best_ang_vel = test_ang_vel;
              best_fpl = detect_result.second;
              best_cost = cost;
              found_alt_direction = true;
            }
          }
        }
      }
      //Couldn't find a best direction (stuck in front of obstacle)
      if(!found_alt_direction || (!std::isnan(best_fpl) && best_fpl <= .81)){
        //Need to break, robot needs time to stop
        best_lin_vel = 0;
        best_ang_vel = 0;
        break;
      }
    }
    //if obstacle, poll different directions (v,w) and get best
    drive(best_lin_vel, 0, 0, 0, 0, best_ang_vel);
    loop_rate.sleep();
    double dist = util::norm(-dest.x(), -dest.y(), 0);
    prev_lin_vel = best_lin_vel;
    prev_ang_vel = best_ang_vel;
  }
  drive(0,0,0,0,0,0); //stop
}
void followlib::PathFinding::drive(double lin_x, double lin_y, double lin_z, double ang_x, double ang_y, double ang_z){
  cobot_msgs::CobotDriveMsg cm;
  cm.v = lin_x;
  cm.w = ang_z;
  /*geometry_msgs::Twist tm;
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
  cmd_vel_pub.publish(tm);*/
  cmd_vel_pub.publish(cm);
  ros::spinOnce();
}
