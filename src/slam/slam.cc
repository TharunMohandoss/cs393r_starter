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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;


#define grid_dim 4
#define granualarity 0.05
#define sigma 0.05
#define sigma_range 2



namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}

void get_raster_table(const Vector2f& odom_loc, const float odom_angle, vector<Vector2f>* scan_ptr, vector< vector<float> >* raster_table_pointer)
{
  //scan_ptr has the point cloud, the raster table would be populated in the raster_table argument

  //first we populate the table with 0 probs
  vector< vector<float> >& raster_table = *raster_table_pointer;
  for (int i = 0; i < grid_dim/granualarity; i++)
  {
    vector<float> temp;
    for (int j = 0; j < grid_dim/granualarity; j++)
    {
        temp.push_back(0);
    }
    raster_table.push_back(temp);
  }
  cout<<"0 prob initialization done"<<endl;

  //iterate through the points in scan_ptr and update the raster_table
  vector<Vector2f>& scan = *scan_ptr;
  for(unsigned int j = 0; j< scan.size(); ++j)
    {

     //the obstacle position
     float obst_x = scan[j].x();
     float obst_y = scan[j].y();

     //get the indices of the obstacle in the raster table
     int raster_table_index_obstacle_x = int((obst_x - odom_loc.x())/granualarity) + int(raster_table.size()/2);
     int raster_table_index_obstacle_y = int((obst_y - odom_loc.y())/granualarity) + int(raster_table.size()/2);

     if(raster_table_index_obstacle_x >= raster_table.size() || raster_table_index_obstacle_y >= raster_table.size() ||
     raster_table_index_obstacle_x < 0 || raster_table_index_obstacle_y <0)
     {
       continue; //since obstacle is out of the grid
     }

     cout<<"obstacle indices: "<<raster_table_index_obstacle_x<<" "<<raster_table_index_obstacle_y<<endl;

     //update prob at obstacle indices
     float s = 2 * sigma * sigma;
     float prob = 1/(M_PI * s); //gaussian value at mean
     raster_table[raster_table_index_obstacle_x][raster_table_index_obstacle_y] = prob;


     //move around the obstacle in 2 directions - along odom_angle and perpendicular to odom_angle (2 more directions for reverse)
    //  for(float step = granualarity; step<=sigma_range*sigma; step +=granualarity) //magnitude of step in each of 4 directions
    //  {
    //    //along odom angle +ive direction
    //    float along_odom_pos_x = obst_x + step * cos(odom_angle);
    //    float along_odom_pos_y = obst_y + step * sin(odom_angle);


    //    //along odom angle -ive direction
    //    float along_odom_neg_x = obst_x - step * cos(odom_angle);
    //    float along_odom_neg_y = obst_y - step * sin(odom_angle);

    //    //perpendicular odom angle +ive direction
    //    float perpend_odom_pos_x = obst_x + step * sin(odom_angle);
    //    float perpend_odom_pos_y = obst_y + step * cos(odom_angle);

    //    //perpendicular odom angle -ive direction
    //    float perpend_odom_neg_x = obst_x - step * sin(odom_angle);
    //    float perpend_odom_neg_y = obst_y - step * cos(odom_angle);
    //  }

    }
    
}


void test_get_raster_table()
{
  cout<<"test raster table called"<<endl;
  Vector2f odom_loc = Vector2f(5, 5);
  float odom_angle = 0.0;
  vector<Vector2f> scan;
  scan.push_back(Vector2f(6,6));
  scan.push_back(Vector2f(5,6));
  vector< vector<float> >* raster_table_pointer;
  get_raster_table(odom_loc, odom_angle, &scan, raster_table_pointer);

  vector< vector<float> >& raster_table = *raster_table_pointer;

  for(int i=0; i<raster_table.size(); ++i)
  {
    for(int j= 0; j<raster_table[i].size();++j)
    cout<<raster_table[i][j]<<" ";

    cout<<endl;
  }

}



void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
