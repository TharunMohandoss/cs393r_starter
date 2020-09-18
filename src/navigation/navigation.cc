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

float Navigation::CalcPointFPL(const Vector2f obstacle, float curvature, float &final_x, float &final_y) { 
	// curvature = 0 
	if(curvature == 0) {
		if(obstacle.y()<=car_width/2 && obstacle.y()>=-car_width/2 && obstacle.x()>0) {
			final_x = obstacle.x() - (car_length+wheel_base)/2;
			final_y = 0;
			if(final_x>2)
			{
				final_x = 2;
				return 2;
			}
			return obstacle.x() - (car_length+wheel_base)/2;
		} else {
			return std::numeric_limits<float>::max();
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

		float theta_max,r_goal, dotproduct;
		r_goal = sqrt(pow(nav_goal_loc_.x(), 2) + pow(nav_goal_loc_.y() - radius, 2));
		dotproduct = -radius * (nav_goal_loc_.y() - radius)/(abs(radius)*(r_goal));
		if(nav_goal_loc_.x()>=0) {
			theta_max = acos(dotproduct/(abs(radius) * r_obstacle));
		} else {
			theta_max = 2 * M_PI - acos(dotproduct/(abs(radius) * r_obstacle));
		}

		if (theta1 > theta_max)
		{
			theta1 = theta_max;
		}

		final_x = radius - radius * cos(theta1);
		final_y = abs(radius) * sin(theta1);
		
		if(rmin <= r_obstacle && r_obstacle <= rs) {
			// point hitting inner side

			theta2 = acos(rmin/r_obstacle);
			return abs(radius * (theta1 - theta2));
			
		} else if(rs <= r_obstacle && r_obstacle <= rmax) {
			// point hitting front side
			
			theta2 = asin((car_length + wheel_base)/(2 * r_obstacle));
			return abs(radius * (theta1 - theta2));
		}

		return std::numeric_limits<float>::max(); 
	}
	
}

float Navigation::CalculateScore(float fpl, float clearance, float goal_dist) {
	return fpl - 100*goal_dist;
}

float Navigation::CalcGoalDistance(float x_goal, float y_goal, float x_final, float y_final) {
	return sqrt(pow(x_goal-x_final, 2) + pow(y_goal-y_final, 2));
}

vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
	float cmin, cmax, step, max_score, max_score_fpl, max_score_curv;

	cmin = -5.0;
	cmax = 5.0;
	step = 1.0;
	max_score = -1;
	max_score_fpl = -1;
	max_score_curv = 0;
	
	for(float c=cmin; c<=cmax; c=c+step){
		float min_fpl = std::numeric_limits<float>::max();
		float min_fpl_x, min_fpl_y;
		float final_x,final_y;
		min_fpl_x = -1;
		min_fpl_y = -1;
		for(unsigned int i=0; i<obstacles.size(); i++){
			float fpl = CalcPointFPL(obstacles[i], c,final_x,final_y);
			if(fpl < min_fpl) {
				min_fpl = fpl;
				min_fpl_x = final_x;
				min_fpl_y = final_y;
			}
		}
		//compare score here
		// might be sketchy
		float distance_goal = CalcGoalDistance(2, 0, min_fpl_x, min_fpl_y);
		if(CalculateScore(min_fpl, 0, distance_goal) > max_score){
			max_score = CalculateScore(min_fpl, 0, distance_goal);
			max_score_fpl = min_fpl;
			max_score_curv = c;
		}
	}
	// probably 1D TOC 
	vector<float> ret_val;
	ret_val.push_back(max_score_fpl);
	ret_val.push_back(max_score_curv);
	return ret_val;
}

void Navigation::Run() {
	vector<float> ret_vals = SelectCurvature(point_cloud_);
	std::cout<<"4\n";
	float u = sqrt(pow(robot_vel_.x(), 2) + pow(robot_vel_.y(), 2));
	float v = OneDTOC(u, 1, 4, -4, ret_vals[0]);
	std::cout<<"5";

	drive_msg_.velocity = v;
	drive_msg_.curvature = ret_vals[1];
	// drive_msg_.velocity = 1;
	// drive_msg_.curvature = 0;
	drive_pub_.publish(drive_msg_);
}

//a_min is -ve i.e maximum decelertion
//u_max is maximum speed allowed
//a_max is +ve i.e maximum acceleration
float Navigation::OneDTOC(float u,float u_max,float a_max,float a_min,float s)
{
	float timestep = 1.0/20;
	float s_min;
	float error_margin = 0.3;
	s_min = u*u/(2*-a_min);
	if (s<s_min-error_margin)
	{
		std::cout<<"in\n";
		return u + u*u/(2*s)*timestep;
		// return -1;
	}
	else if(u<u_max)//case 1 where the car isn't in maximum speed
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
