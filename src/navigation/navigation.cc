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
#include <algorithm>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
//
using geometry::line2f;
//
using namespace math_util;
using namespace ros_helpers;

#define GRID_RES 0.25

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
const float clearance_length = 0.2;
const float clearance_width = 0.1;
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
	// nav_goal_loc_ = loc;
	// nav_goal_angle_ = angle;
	map_.Load("maps/GDC1.txt");
	cout<<"firstin\n";
  	// const uint32_t kColor = 0xd67d00;
	Node start(0,0);
	Node goal(3,4);
	vector<pair<int,int>> answer;
	Astar(map_,start,goal,answer);
	// visualization::ClearVisualizationMsg(global_viz_msg_);
	for(int i =0;i<((int)answer.size());i++)
	{
		cout<<answer[i].first<<","<<answer[i].second<<endl;
	}
	// viz_pub_.publish(global_viz_msg_);
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
	// std::cout<<"robot_loc_.x() : "<<robot_loc_.x()<<", robot_loc_.y() : "<<robot_loc_.y()<<"robot_angle_ : "<<robot_angle_<<"\n";
}

void Navigation::ObservePointCloud(const vector<Vector2f> cloud,
                                   double time) {
	point_cloud_ = cloud;
}

float Navigation::CalcPointFPL(const Vector2f obstacle, float curvature, float &final_x, float &final_y, bool debug) { 
	// curvature = 0 
	if(abs(curvature) < 0.0001) {
		// std::cout<<"alpha";
		if(obstacle.y()<=(car_width+2*clearance_width)/2 && obstacle.y()>=-(car_width+2*clearance_width)/2 && obstacle.x()>0) {
			final_x = obstacle.x() - ((car_length+2*clearance_length)+wheel_base)/2;
			// if(debug)
			// {
			// 	std::cout<<"obstacle y : "<<obstacle.y()<<",obstacle x : "<<obstacle.x()<<"((car_length+2*clearance_length)+wheel_base)/2 : "<<((car_length+2*clearance_length)+wheel_base)/2<<"\n";
			// }
			final_y = nav_goal_loc_.y();
			if(final_x>nav_goal_loc_.x())
			{
				final_x = nav_goal_loc_.x();
				// if(debug)
				// {
				// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
				// }
				return nav_goal_loc_.x();
			}
			// if(debug)
			// {
			// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
			// }
			return obstacle.x() - ((car_length+2*clearance_length)+wheel_base)/2;
		} else {
			final_x = nav_goal_loc_.x();
			final_y = nav_goal_loc_.y();
			// if(debug)
			// {
			// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
			// }
			return nav_goal_loc_.x();
		}
	} else {
		// if(debug)
		// {
		// 	visualization::DrawCross(Vector2f(obstacle.x(),obstacle.y()),.2, 0xFF0000, local_viz_msg_);
		// }
		// std::cout<<"beta";
		float radius = 1/curvature;
		float rmax, rs, rmin, r_obstacle;
		rmax = sqrt(pow(((car_length+2*clearance_length) + wheel_base)/2, 2) + pow((car_width+2*clearance_width)/2 + abs(radius), 2));
		rmin = abs(radius) - (car_width+2*clearance_width)/2;
		rs = sqrt(pow(((car_length+2*clearance_length) + wheel_base)/2, 2) + pow(abs(radius) - (car_width+2*clearance_width)/2, 2));
		r_obstacle = sqrt(pow(obstacle.x(), 2) + pow(obstacle.y() - radius, 2));
		// if(debug)
		// {
		// 	std::cout<<"rmax : "<<rmax<<", rmin : "<<rmin<<", r_obstacle : "<<r_obstacle<<", radius : "<<radius<<", rs : "<<rs<<"\n";
		// }
		
		// generic for both the directions
		float theta1, theta2, dotproduct1;
		dotproduct1 = -radius * (obstacle.y() - radius);
		if(obstacle.x()>=0) {
			theta1 = acos(dotproduct1/(abs(radius) * r_obstacle));
		} else {
			theta1 = 2 * M_PI - acos(dotproduct1/(abs(radius) * r_obstacle));
		}

		// float theta_max, dotproduct;
		// bool in =false;
		// // r_goal = sqrt(pow(nav_goal_loc_.x(), 2) + pow(0 - radius, 2));
		// dotproduct = abs(radius)/sqrt((radius)*(radius) + nav_goal_loc_.x()*nav_goal_loc_.x());
		// if(nav_goal_loc_.x()>=0) {
		// 	theta_max = acos(dotproduct);
		// } else {
		// 	theta_max = 2 * M_PI - acos(dotproduct);
		// }

		// // std::cout<<"Theta max: "<<theta_max<<", Theta_1: "<<theta1<<"\n";

		// if (theta1 > theta_max)
		// {
		// 	in = true;
		// 	theta1 = theta_max;
		// }


		
		// if(debug)
		// {
		// 	// std::cout<<"1_rmax : "<<rmax<<", rmin : "<<rmin<<", r_obstacle : "<<r_obstacle<<", radius : "<<radius<<", rs : "<<rs<<"\n";
		// }
		if(rmin <= r_obstacle && r_obstacle <= rs) {
			// point hitting inner side

			theta2 = acos(rmin/r_obstacle);
			// if(in)
			// {
			// 	theta1 = theta1 + theta2;
			// }

			// std::cout<<"1_rmax : "<<rmax<<", rmin : "<<rmin<<", r_obstacle : "<<r_obstacle<<", radius : "<<radius<<", rs : "<<rs<<"\n";
			// if(debug)
			// {
			// 	std::cout<<"in1  : Theta1: "<<theta1<<", Theta2: "<<theta2<<", X: "<<obstacle.x()<<", Y: "<<obstacle.y()<<"\n";
			// 	std::cout<<"retval : "<<abs(radius * (theta1 - theta2))<<"\n";
			// }
			// std::cout<<"Theta1: "<<theta1<<", Theta2: "<<theta2<<", X: "<<obstacle.x()<<", Y: "<<obstacle.y()<<"\n";
			final_y = radius - radius * cos(theta1-theta2);
			final_x = abs(radius) * sin(theta1-theta2);
			// if(debug)
			// {
			// 	// float start_angle = 3.14/2
			// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
			// 	visualization::DrawArc(Vector2f(0, radius), abs(radius),-3.14,3.14, 0x3b72d3, local_viz_msg_);
			// }
			return abs(radius * (theta1 - theta2));

			
		} else if(rs <= r_obstacle && r_obstacle <= rmax) {
			// point hitting front side
			// std::cout<<"2_rmax : "<<rmax<<", rmin : "<<rmin<<", r_obstacle : "<<r_obstacle<<", radius : "<<radius<<", rs : "<<rs<<"\n";
			// std::cout<<"Theta1: "<<theta1<<", Theta2: "<<theta2<<", X: "<<obstacle.x()<<", Y: "<<obstacle.y()<<"\n";
			
			theta2 = asin(((car_length+2*clearance_length) + wheel_base)/(2 * r_obstacle));
			// if(in)
			// {
			// 	theta1 = theta1 + theta2;
			// }
			final_y = radius - radius * cos(theta1-theta2);
			final_x = abs(radius) * sin(theta1-theta2);
			// if(debug)
			// {
			// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
			// 	visualization::DrawArc(Vector2f(0, radius), abs(radius),-3.14,3.14, 0x3b72d3, local_viz_msg_);
			// }
			// if(debug)
			// {
			// 	std::cout<<"in2  : Theta1: "<<theta1<<", Theta2: "<<theta2<<", X: "<<obstacle.x()<<", Y: "<<obstacle.y()<<"\n";
			// 	std::cout<<"retval : "<<abs(radius * (theta1 - theta2))<<"\n";
			// }
			return abs(radius * (theta1 - theta2));
		} else {
			final_x = 0;
			final_y = 0;
			// if(debug)
			// {
			// 	visualization::DrawCross(Vector2f(final_x,final_y),.2, 0x00FF00, local_viz_msg_);
			// 	visualization::DrawArc(Vector2f(0, radius), abs(radius),-3.14,3.14, 0x3b72d3, local_viz_msg_);
			// }
			// if(debug)
			// {
			// 	std::cout<<"in3 \n";
			// 	std::cout<<"retval : "<<abs(radius * 6.28)<<"\n";
			// }
			return abs(radius * 6.28); 
		}

		 
	}
	
}	

float Navigation::CalculateScore(float fpl, float clearance, float goal_dist) {
	return 10*fpl-goal_dist;
}

float Navigation::CalcGoalDistance(float x_goal, float y_goal, float x_final, float y_final) {
	return sqrt(pow(x_goal-x_final, 2) + pow(y_goal-y_final, 2));
}

vector<float> Navigation::SelectCurvature(vector<Vector2f> obstacles){
	// float cmin, cmax, step, 
	float max_score, max_score_fpl, max_score_curv;
	float curvatures[] = {-6,-5,-4,-3,-2.5,-2,-1.5,-0.9,-0.7,-0.5,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.5,0.7,0.9,1.5,2,2.5,3,4,5,6};

	// cmin = -6.0;
	// cmax = 6.0;
	// step = 0.3;
	max_score = -std::numeric_limits<float>::max();
	// std::cout<<"min : "<<max_score<<"\n";
	max_score_fpl = -1;
	max_score_curv = 0;
	// std::cout<<"selecting curvature\n";
	
	// for(float c=cmin; c<=cmax; c=c+step){
	// 	// visualization::ClearVisualizationMsg(local_viz_msg_);
	// 	if(c!=0)
	// 	{
	// 		visualization::DrawArc(Vector2f(0, 1/c), abs(1/c),-3.14,3.14, 0x3b72d3, local_viz_msg_);
	// 	}
	// 	// viz_pub_.publish(local_viz_msg_);
	// }

	// for(float c=cmin; c<=cmax; c=c+step){
	for(unsigned int current=0; current<sizeof(curvatures)/sizeof(float); current++){
		float c = curvatures[current];
		// std::cout<<"curvature : "<<c<<",max_score_fpl : "<<max_score_fpl<<"\n";
		float min_fpl = std::numeric_limits<float>::max();
		float min_fpl_x, min_fpl_y;
		float final_x,final_y;
		min_fpl_x = -1;
		min_fpl_y = -1;
		//int min_obstacle = -1;
		for(unsigned int i=0; i<obstacles.size(); i++){
			float fpl = CalcPointFPL(obstacles[i], c,final_x,final_y,false);
			// std::cout<<"i : "<<i<<", fpl : "<<fpl<<", min_fpl : "<<min_fpl<<"\n";
			//cout<<obstacles[i].x()<<" "<<obstacles[i].y()<<" "<<final_x<<" "<<final_y<<endl;
			//we priortize avoiding obstalces near by
                        float obst_dist = sqrt(pow(obstacles[i].x()-final_x, 2) + pow(obstacles[i].y()-final_y, 2));
			fpl *= obst_dist; //nearby obstalces will have a lower score, hence be selecyted in the next if statement
			if(fpl <= min_fpl) {
				//min_obstacle = i;
				min_fpl = fpl;
				min_fpl_x = final_x;
				min_fpl_y = final_y;
			}
		}
		/*
		if (min_obstacle!=-1)
		{
			// std::cout<<"c : "<<c<<"min_i : "<<min_obstacle<<"\n";
			// std::cout<<"obstacle x : "<<obstacles[min_obstacle].x()<<", obstacle y : "<<obstacles[min_obstacle].y()<<"\n";
			CalcPointFPL(obstacles[min_obstacle], c,final_x,final_y,true);
		}*/
		//compare score here
		// might be sketchy
		// std::cout<<"min_fpl_x : "<<min_fpl_x<<", "<<min_fpl_y<<"\n";
		float distance_goal = CalcGoalDistance(nav_goal_loc_.x(), nav_goal_loc_.y(), min_fpl_x, min_fpl_y);
		float cur_score = CalculateScore(min_fpl, 0, distance_goal);
		// std::cout<<"min_fpl_x : "<<min_fpl_x<<", min_fpl_y : "<<min_fpl_y<<"\n";
		// std::cout<<"curvature : "<<c<<" cur_score: "<<cur_score<<", max_score : "<<max_score<<" max_score_c : "<<max_score_curv<<"\n";
		// std::cout<<"distance_goal : "<<distance_goal<<", min_fpl : "<<min_fpl<<"\n\n";
		if(cur_score > max_score){
			max_score = cur_score;
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
	nav_goal_loc_ = Vector2f(10,0);
	// float v = OneDTOC(prev_velocity, 1, 4, -4, ret_vals[0]);
	// prev_velocity = v;

	drive_msg_.velocity = 0.5;
	// drive_msg_.curvature = ret_vals[1];
	// drive_msg_.velocity = 0.1;
	drive_msg_.curvature = -8;
	drive_pub_.publish(drive_msg_);
}

//a_min is -ve i.e maximum decelertion
//u_max is maximum speed allowed
//a_max is +ve i.e maximum acceleration
float Navigation::OneDTOC(float u,float u_max,float a_max,float a_min,float s)
{
	float timestep = 1.0/20;
	float s_min;
	float error_margin = 0;
	s_min = u*u/(2*-a_min);
	if(s<0.02)
	{
		return 0;
	}
	else if (s<=s_min-error_margin)
	{
		// std::cout<<"in\n";
		float out = u - (u*u/(2*s))*timestep;
		if(out<0)
		{
			std::cout<<"in2";
			out = 0;
		}
		return out;
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

// i is along x axis and j is along y axis
void Navigation::CalculateGrid(Vector2f xy, int& i, int& j) {
	float x, y;
	x = xy.x();
	y = xy.y();

	i = floor(x/GRID_RES);
	j = floor(y/GRID_RES);
}

}  // namespace navigation



































































bool operator == (Node const &  first,Node const & second)
{
	return (first.i==second.i) && (first.j==second.j);
}
Node::Node(int i, int j) {
	this->i = i;
	this->j = j; 
	this->h = 0;
	this->g = 0;
	//need to implement distance from end point as h
}
Node::Node(int i, int j,const Node& goal) {
	this->i = i;
	this->j = j; 
	this->g = 0;

	float diff_i = abs(goal.i - this->i);
	float diff_j = abs(goal.j - this->j);
	// float min_diff = diff_i;
	// if(diff_j<min_diff)
	// {
	// 	min_diff = diff_j;
	// }
	// float max_diff = diff_j + diff_i - min_diff;
	// this->h = min_diff*1.414 + (max_diff-min_diff);
	this->h = diff_j + diff_i;
	//need to implement distance from end point as h
}
bool Node::NodeIntersectsMap(vector<line2f> map_lines) {

	for(unsigned int i = 0; i < map_lines.size(); i++) {
		line2f map_line = map_lines[i];
		float left_x, right_x, top_y, bottom_y;
		left_x = this->i * GRID_RES;
		right_x = (this->i + 1) * GRID_RES;
		bottom_y = this->j * GRID_RES;
		top_y = (this->j + 1) * GRID_RES;
		const line2f line1(left_x, bottom_y, left_x, top_y);
		const line2f line2(left_x, top_y, right_x, top_y);
		const line2f line3(right_x, top_y, right_x, bottom_y);
		const line2f line4(right_x, bottom_y, left_x, bottom_y);

		if(map_line.Intersects(line1) || map_line.Intersects(line2) ||
			map_line.Intersects(line3) || map_line.Intersects(line4)) {
			return true;
		}
	}

	return false;

}
float Node::GetValue(vector<line2f> map_lines) {
	return 0;
	// float min_dist = std::numeric_limits<float>::max();
	// for(unsigned int i=0; i<map_lines.size(); i++) {
	// 	line2f line = map_lines[i];
	// 	Vector2f point( (this->i+0.5) * GRID_RES, (this->j+0.5) * GRID_RES);
	// 	float dist_points = pow(line.p0.x()-line.p1.x(), 2) + pow(line.p0.y()-line.p1.y(), 2);

		 
		
	// 	if(dist_points == 0) {
	// 		return -(pow(line.p0.x()-point.x(), 2) + pow(line.p0.y()-point.y(), 2));
	// 	}
	// 	float p0_point_x = point.x() - line.p0.x();
	// 	float p0_point_y = point.y() - line.p0.y();

	// 	float p0_p1_x = line.p1.x() - line.p0.x();
	// 	float p0_p1_y = line.p1.y() - line.p0.y();
	// 	float dot = p0_p1_x * p0_point_x + p0_p1_y * p0_point_y;
	// 	const float t = max((float)0.0, min((float)1.0, dot/dist_points));

	// 	float x_proj = line.p0.x() + t * p0_p1_x;
	// 	float y_proj = line.p0.y() + t * p0_p1_y;

	// 	float dist = (pow(x_proj - point.x(), 2) + pow(y_proj - point.y(), 2));

	// 	if(dist<min_dist) {
	// 		min_dist = dist;
	// 	}
		
	// }
	// return -min_dist;
}
float Node::GetPriority()
{
	return -this->g - this->h;
}
#define WEIGHT_COST 1
// #include<iostream>
// #include<pair>
// #include "node.h"
#include "simple_queue.h"

#ifndef ASTAR
#define ASTAR

//returns 1 if path found
int Astar(vector_map::VectorMap map_,Node start_point,Node& end_point,vector<pair<int,int>>& answer)
{
	SimpleQueue<Node,float> open_queue;
	SimpleQueue<Node,float> closed_queue;
		node_map[q.GetState()] = q;
	open_queue.Push(start_point,start_point.GetPriority());
	unordered_map<string, Node> node_map;

	while(!open_queue.Empty())
	{
		cout<<"inA\n";
		Node q = open_queue.Pop();
		
		
		Node succesors[8];
		int succesors_x[] = {-1,-1,0,1,1,1,0,-1};//clockwise starting from left
		int succesors_y[] = {0,1,1,1,0,-1,-1,-1};
		float distances[] = {1,1.414,1,1.414,1,1.414,1,1.414};
		for(int neigh=0;neigh<8;neigh++)
		{
			Node cur_succesor = Node(q.i + succesors_x[neigh],q.j+succesors_y[neigh],end_point);
			succesors[neigh] = cur_succesor;
		}
		int i = 0;
		//potent
		for(auto& succesor : succesors)
		{
			if(succesor.NodeIntersectsMap(map_.lines))
			{
				continue;
			}
			if(succesor==end_point)
			{
				end_point.parent = &q;
				Node cur_node = end_point;
				int count = 0;
				while(!(cur_node == start_point))
				{
					if(count==4)
					{
						break;
					}
					count++;
					pair<int,int> node_pair(cur_node.i,cur_node.j);
					cout<<"inside loop, i : "<<cur_node.i<<", j : "<<cur_node.j<<endl;
					answer.push_back(node_pair);
					cur_node = *(cur_node.parent);
				}
				pair<int,int> node_pair(cur_node.i,cur_node.j);
				answer.push_back(node_pair);
				return 1;
			}
			succesor.g = q.g + WEIGHT_COST*succesor.GetValue(map_.lines) + distances[i];//to implement cost as value?
			// succesor.h = abs(end_point.i-succesor.i) + abs(end_point.j-succesor.j);//what cost is best?
			if(open_queue.Exists(succesor))
			{
				float cur_priority = open_queue.GetPriority(succesor);
				if(succesor.GetPriority() > cur_priority)
				{
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
			}
			else if(closed_queue.Exists(succesor))
			{
				float cur_priority = closed_queue.GetPriority(succesor);
				if(succesor.GetPriority() > cur_priority)
				{
					continue;
					// open_queue.push(succesor,succesor.g+succesor.h);
				}
			}
			else
			{	
				succesor.parent = &q;
				cout<<"parent, i : "<<succesor.i<<",j : "<<succesor.j<<", parent is i : "<<succesor.parent->i<<", j : "<<succesor.parent->j<<endl;
				open_queue.Push(succesor,succesor.GetPriority());
				Node new_succ = open_queue.Search(succesor);
				// cout<<"newparent, i : "<<new_succ.i<<",j : "<<new_succ.j<<", parent is i : "<<new_succ.parent->i<<", j : "<<new_succ.parent->j<<endl;
				// Node new_succ2 = open_queue.Pop();
				// cout<<"newparent, i : "<<new_succ2.i<<",j : "<<new_succ2.j<<", parent is i : "<<new_succ2.parent->i<<", j : "<<new_succ2.parent->j<<endl;
				// exit(0);
			}

			i++;
		}
		closed_queue.Push(q,q.GetPriority());

	}
	return 0;
}	



#endif  // ASTAR
