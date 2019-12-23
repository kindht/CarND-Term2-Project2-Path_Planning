#include "trajectory.h"

vector<vector<double>> Generate_Trajectory(const MyCar &my_car, const CarState &car_state, 
                         const MapWayPoints &map_waypoints, const PreviousPath &previous_path){
    /*
    Generate trajectory for my car to travel
    @param my_car,    info of my car (position, yaw etc.)
    @param car_state, data got from planning
    @map_waypoints,   waypoints on map
    @previous_path,   previous path

    @return next path, {next_x_vals, next_y_vals}
    */
    
    // Retrieve  my car's info
    double car_x = my_car.x;
    double car_y = my_car.y;
    double car_s = my_car.s;
    double car_yaw = my_car.yaw;
    
    int lane = car_state.lane;
    double target_speed = car_state.target_speed;
    bool ChangeLane = car_state.ChangeLane;

    // Retrieve map_waypoints (x, y, s)
    vector<double> map_waypoints_x = map_waypoints.map_waypoints_x;
    vector<double> map_waypoints_y = map_waypoints.map_waypoints_y;
    vector<double> map_waypoints_s = map_waypoints.map_waypoints_s;
  
    // Previous path points (x, y)
    vector<double> previous_path_x = previous_path.previous_path_x;
    vector<double> previous_path_y = previous_path.previous_path_y;

    // Trajectory points
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // ref point as the origin of car/local coordinates
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // a list of widely spaced (x,y) points in order to create a spline
    vector<double> ptsx;
    vector<double> ptsy;
            
    // if previous path almost empty , using car as starting reference
    int prev_size = previous_path_x.size();
    if (prev_size < 2) { 
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } 
    // use the previous path's end point as starting reference
    else {  
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    } // End if prev_size

    // Find more points that will be used for creating a spline
    vector<double> next_point(2);
    double dist = 30.0;
    if (ChangeLane) { dist = 40;} // larger dist for smooth lane change
    
    for (int i = 1; i <= 3; ++i) {
        next_point = getXY((car_s + dist * i), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        ptsx.push_back(next_point[0]);
        ptsy.push_back(next_point[1]);
    } 

    // Convert points from map/global coordinates to car/local coordinates  
    // Take (ref_x, ref_y, ref_yaw)  as reference point 
    for (int i = 0; i < ptsx.size(); ++i) {      

        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw)); 
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw)); 
    }
    
    // Create a spline
    tk::spline s;
    // Set points to the spline
    s.set_points(ptsx, ptsy);
  
    // Add points from previous_path to future path  
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points to travel at desired reference velocity
    // (target_x , target_y)  in car coordinates with origin at (ref_x, ref_y, ref_yaw)
    double target_x = 30.0; 
    if (ChangeLane) { target_x = 40;}
    
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    
    double x_start = 0;
   
    double N = (target_dist/(0.02*target_speed));
    double offset = (target_x) / N;

    //Fill up the rest of path after filling it with previous path points, total 50 points
    for (int i = 1; i <= 50-previous_path_x.size(); ++i) {

        double x_point = x_start + offset;
        double y_point = s(x_point);
        x_start = x_point;

        // rotate back to global coordinates (ref_x , ref_y are in map/global coordinates)
        double x_global = ref_x +  (x_point * cos(ref_yaw) - y_point*sin(ref_yaw));
        double y_global = ref_y +  (x_point * sin(ref_yaw) + y_point*cos(ref_yaw));

        next_x_vals.push_back(x_global);
        next_y_vals.push_back(y_global);

    } // End for 
  
    return {next_x_vals, next_y_vals};

}// End Generate_Trajectory
