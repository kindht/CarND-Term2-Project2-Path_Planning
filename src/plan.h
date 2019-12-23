#ifndef PLAN_H
#define  PLAN_H

#include <iostream> // Debug

#include <string>
#include <vector>
#include <iterator>
#include <map>

#include "Car.h"
#include "cost.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;

CarState Plan(const map<string, CloseCar> &predictions, double target_speed, int lane);

double Update_target_speed(double ahead_dist, double ahead_speed, double target_speed);

#endif //PLAN_H
