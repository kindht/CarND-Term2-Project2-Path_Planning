#ifndef TRAJECTORY_H
#define  TRAJECTORY_H

#include <iostream>  // Debug
#include <cmath>
#include <vector>

#include "spline.h"
#include "helpers.h"
#include "Car.h"

using std::vector;
using std::cout;
using std::endl;


struct MapWayPoints {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

struct PreviousPath {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
};

vector<vector<double>> Generate_Trajectory(const MyCar &my_car, const CarState &car_state, 
                         const MapWayPoints &map_waypoints, const PreviousPath &previous_path);


#endif //TRAJECTORY_H