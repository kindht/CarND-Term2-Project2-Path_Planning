#ifndef COST_H
#define  COST_H

#include <algorithm>
#include <cmath>

// Weights of costs
const double W_DISTANCE = 50; 
const double W_SPEED    = 40;

const double SPEED_LIMIT = 22.3; // 50mph ~= 22.3 m/s
const double BUFFER_V = 0.2;     // max_target_speed = 49.5 mph ~= 22.1 m/s

const double BUFFER_DIST = 35;   // 35m Safty distance


double Calculate_cost(double dist_ahead, double dist_behind, double ahead_speed, double my_speed);

double Calc_distance_cost(double dist);

double Calc_speed_cost(double ahead_speed, double my_speed);

#endif // COST_H 