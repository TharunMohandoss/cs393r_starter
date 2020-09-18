//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <limits>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;



namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

const float car_width = 0.281;
const float car_length = 0.535;
const float car_height = 0.15;
const float wheel_base = 0.324;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	nav_goal_loc_ = loc;
	nav_goal_angle_ = angle;
}


void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	// not being called
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
	robot_loc_ = loc;
	robot_angle_ = angle;
	robot_vel_ = vel;
	robot_omega_ = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
	point_cloud_ = cloud;
}

float Navigation::CalcPointFPL(const Vector2f obstacle, float curvature) {

	// curvature = 0 
	if(curvature == 0) {
		if(obstacle.y()<=car_width/2 && obstacle.y()>=-car_width/2 && obstacle.x()>0) {
			return obstacle.x() - (car_length+wheel_base)/2;
		} else {
			return -1;
		}
	} else {
		float radius = 1/curvature;
		float rmax, rs, rmin, r_obstacle;
		rmax = sqrt(pow((car_length + wheel_base)/2, 2) + pow(car_width/2 + abs(radius), 2));
		rmin = abs(radius) - car_width/2;
		rs = sqrt(pow((car_length + wheel_base)/2, 2) + pow(abs(radius) - car_width/2, 2));
		r_obstacle = sqrt(pow(obstacle.x(), 2) + pow(obstacle.y() - radius, 2));

		// generic for both the directions
		float theta1, theta2, dotproduct1;
		dotproduct1 = -radius * (obstacle.y() - radius);
		if(obstacle.x()>=0) {
			theta1 = acos(dotproduct1/(abs(radius) * r_obstacle));
		} else {
			theta1 = 2 * M_PI - acos(dotproduct1/(abs(radius) * r_obstacle));
		}
		
		if(rmin <= r_obstacle && r_obstacle <= rs) {
			// point hitting inner side

			theta2 = acos(rmin/r_obstacle);
			return abs(radius * (theta1 - theta2));
			
		} else if(rs <= r_obstacle && r_obstacle <= rmax) {
			// point hitting front side
			
			theta2 = asin((car_length + wheel_base)/(2 * r_obstacle));
			return abs(radius * (theta1 - theta2));
		}

		return -1; 
	}
	
}

float Navigation::CalculateScore(float fpl, float clearance, float goal_dist) {
	return fpl - goal_dist;
}

vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
	float cmin, cmax, step, max_fpl, max_fpl_curv, goal_dist;

	cmin = -5.0;
	cmax = 5.0;
	step = 1.0;
	max_fpl = -1;
	max_fpl_curv = 0;
	for(float c=cmin; c<=cmax; c=c+step){
		float min_fpl = std::numeric_limits<float>::max();
		for(unsigned int i=0; i<obstacles.size(); i++){
			float fpl = CalcPointFPL(obstacles[i], c);
			if(fpl < min_fpl) {
				min_fpl = fpl;
			}
		}
		//compare score here
		if(CalculateScore(min_fpl, ) > CalculateScore(max_fpl, )){
			max_fpl = min_fpl;
			max_fpl_curv = c;
		}
	}
	// probably 1D TOC 
	vector<float> ret_val;
	ret_val[0] = max_fpl;
	ret_val[1] = max_fpl_curv;
	return ret_val;
}

void Navigation::Run() {

	// drive_msg_.velocity = 1.0;
	// drive_msg_.curvature = 0.0;
	// drive_pub_.publish(drive_msg_);
}

//a_min is -ve i.e maximum decelertion
//u_max is maximum speed allowed
//a_max is +ve i.e maximum acceleration
float 1DTOC(float u,float u_max,float a_max,float a_min,float s)
{
	float timestep = 1.0/20;
	float s_min;
	float error_margin = 0.3;
	s_min = u*u/(2*-a_min);
	if (s<s_min-error_margin)
	{
		return -1;
	}
	else if(u<u_maxu)//case 1 where the car isn't in maximum speed
	{
		return u + a_max*timestep;
	}
	else if(s>s_min)//case 2 where car is in maximum speed and doesn't require to decelerate yet
	{
		return u;
	}
	else//case 3 where car needs to decelerate
	{
		return u + a_min*timestep;//decelrated value
	}
}

}  // namespace navigation
