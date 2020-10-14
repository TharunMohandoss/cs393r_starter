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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"


#include "amrl_msgs/AckermannCurvatureDriveMsg.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

// namespace {
// ros::Publisher drive_pub_;
// ros::Publisher viz_pub_;
// VisualizationMsg local_viz_msg_;
// VisualizationMsg global_viz_msg_;
// AckermannCurvatureDriveMsg drive_msg_;
// // Epsilon value for handling limited numerical precision.
// const float kEpsilon = 1e-5;

// const float car_width = 0.281;
// const float car_length = 0.535;
// const float car_height = 0.15;
// const float wheel_base = 0.324;
// const float clearance_length = 0.2;
// const float clearance_width = 0.1;
// } 

DEFINE_double(num_particles, 50, "Number of particles");
#define k1_x 1
#define k2_x 1
#define k1_y 1
#define k2_y 1
#define k3_theta 1
#define k4_theta 1
#define lidar_dist 0.2
#define sigma 1000.0
#define d_long 0.5
#define d_short 0.5
#define s_min 0.2
#define s_max 3.0

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles, vector<Vector2f>* our_obstacles) const {
  *particles = particles_;
  *our_obstacles = our_obstacles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
 cout<<"in getpredpointcloud"<<endl;
  vector<Vector2f>& scan = *scan_ptr;
  float point_x, point_y, point_theta;
  point_x = loc.x();
  point_y = loc.y();
  point_theta = angle;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
 //cout<<"-1"<<endl;
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  // for (size_t i = 0; i < scan.size(); ++i) {
  //   scan[i] = Vector2f(0, 0);
  // }
  //cout<<"0"<<endl;

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  float laser_x, laser_y;
  laser_x = point_x + lidar_dist * cos(point_theta);
  laser_y = point_y + lidar_dist * sin(point_theta);
  Vector2f laser_vector(laser_x, laser_y);

  //cout<<"1"<<endl;

  for (size_t angle_i = 0; angle_i < scan.size(); ++angle_i) {

    float angle_ls, final_x, final_y, final_distance; 
    angle_ls = point_theta + angle_min + angle_i * (angle_max-angle_min)/num_ranges;
    final_x = laser_x + range_max * cos(angle_ls);
    final_y = laser_y + range_max * sin(angle_ls);
    final_distance = range_max;

    for (size_t map_i = 0; map_i < map_.lines.size(); ++map_i) {
      const line2f map_line = map_.lines[map_i];
      // The line2f class has helper functions that will be useful.
      // You can create a new line segment instance as follows, for :
      // line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
      // Access the end points using `.p0` and `.p1` members:
      const line2f laser_line(laser_x, laser_y, final_x, final_y); 
      // printf("P0: %f, %f P1: %f,%f\n", 
      //        my_line.p0.x(),
      //        my_line.p0.y(),
      //        my_line.p1.x(),
      //        my_line.p1.y());

      // Check for intersections:
      bool intersects = map_line.Intersects(laser_line);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      if(intersects) { 
        std::cout<<"Intersected"<<"\n";
        Vector2f intersection_point;
        intersects = map_line.Intersection(laser_line, &intersection_point);
        float distance_intersection;
        distance_intersection = (laser_vector-intersection_point).norm();
        if(distance_intersection<final_distance) {
          final_distance = distance_intersection;
          final_x = intersection_point.x();
          final_y = intersection_point.y();
          std::cout<<"Updated"<<"\n";
        }
      }
      // if (intersects) {
      //   printf("Intersects at %f,%f\n", 
      //          intersection_point.x(),
      //          intersection_point.y());
      // } else {
      //   printf("No intersection\n");
      // }
    }
    if(final_distance<range_min) {
      final_x = laser_x + range_min * cos(angle_ls);
      final_y = laser_y + range_min * sin(angle_ls);
      final_distance = range_min;
    }
    Vector2f final_intersection(final_x, final_y);
    scan[angle_i] = final_intersection;
  }
  
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
   
  vector<Vector2f> scan_ptr;

  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(),
                                            range_min,
                                            range_max,
                                            angle_min,
                                            angle_max,
                                            &scan_ptr);


  //computing distance from obstacle
  vector<float> point_distances;
  for(unsigned int i = 0; i< ranges.size(); ++i)
  {

   //the obstacle position
   float obst_x = scan_ptr[i].x();
   float obst_y = scan_ptr[i].y();

   float loc_x = p_ptr->loc.x();
   float loc_y = p_ptr->loc.y();

   float distance = sqrt(pow((obst_x - loc_x), 2) +  pow((obst_y - loc_y), 2));
   //cout<<distance<<endl;
   point_distances.push_back(distance);
  }

  //computing the L2 distance between point distances and scan_ptr
  float l2_distance_square = 0.0;
  for(unsigned int j = 0; j< ranges.size(); ++j)
    {
      float w;

      //robust optimal likelihood
      if(ranges[j] < s_min || ranges[j] > s_max)
        w = 0;
      else if (ranges[j] < point_distances[j] - d_short)
       w = pow(d_short, 2);
      else if (ranges[j] > point_distances[j] + d_long)
       w = pow(d_long, 2);
      else
      w = pow(point_distances[j] - ranges[j], 2);

     l2_distance_square += w;
     
    }

  //computing weight
  
  float weight = exp(-l2_distance_square/(2 * sigma));

  p_ptr->weight = weight;

}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  
  
  
  
  //appending dummy points to particles_ for testing (COMMENT THIS BLOCK IF NOT TESTING)
  //Particle p1 = {Vector2f(0, 0), 0, 0};
  //Particle p2 = {Vector2f(10, 10), 2.0, 0};
  //particles_.push_back(p1);
 // particles_.push_back(p2);
  //cout<<particles_.size()<<endl;


   float total_weight = 0.0;
   for(unsigned int i=0; i<particles_.size();++i)
   {
   Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
   total_weight += (&particles_[i])->weight; //calculating sum of updated weights
   }

  //normalizing weights
   for(unsigned int i=0; i<particles_.size();++i)
   (&particles_[i])->weight /= total_weight;

  //for(unsigned int i=0; i<particles_.size();++i)
 // cout<<(&particles_[i])->weight<<endl;
 // exit(0);
 
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  // std::cout<<"map file  : "<<map_file<<"\n";
  map_.Load("maps/GDC1.txt");

  // visualization::ClearVisualizationMsg(local_viz_msg_);
  robot_x = loc.x();
  robot_y = loc.y();
  robot_angle = angle;
  // std::cout<<"robot_angle: "<<robot_angle<<"\n";



  // for(unsigned int i=0; i<FLAGS_num_particles; i++) {
  for (unsigned int i=0; i<1; i++) {
      Particle particle;
      Vector2f predict_loc;
      // replace 0
      predict_loc.x() = robot_x + rng_.Gaussian(0.0, k1_x * sqrt(1) + k2_x * abs(2) );
      predict_loc.y() = robot_y + rng_.Gaussian(0.0, k1_y * sqrt(3) + k2_y * abs(4) );

      // visualization::DrawCross(loc,2, 0xFF0000, local_viz_msg_);

      particle.loc = predict_loc;
      particle.angle = robot_angle + rng_.Gaussian(0, k3_theta * sqrt(5) + k4_theta * abs(6));
      particle.weight = 1/FLAGS_num_particles;
      particles_.push_back(particle);
      // std::cout<<"pushed : "<<particles_.size()<<"\n";

      // GetPredictedPointCloud(predict_loc, particle.angle, 1000, 100.0, 0.0, -3.14, 3.14, &our_obstacles_);
  }
  // viz_pub_.publish(local_viz_msg_);


}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
