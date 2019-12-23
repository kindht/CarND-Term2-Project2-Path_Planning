#ifndef PREDICTION_H
#define  PREDICTION_H

#include <cmath> // <math.h> works
#include <vector>
#include <string>
#include <map>

#include "Car.h"

using std::vector;
using std::string;
using std::map;

map<string, CloseCar> Predict(const vector<vector<double>> &sensor_fusion, double car_s, int prev_size, int lane);

#endif //PREDICTION_H