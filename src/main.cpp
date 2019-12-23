#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"

// my headers
#include "prediction.h"
#include "plan.h"
#include "trajectory.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using std::cout;
using std::endl;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Init a target lane,  0/1/2 = left/center/right 
  int lane = 1; 
  // Init a target speed  
  double target_speed = 0.0; 

  h.onMessage([&lane, &target_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Get previous path size
          int prev_size = previous_path_x.size(); 

          if (prev_size > 0) {
            car_s = end_path_s; // if previous path exits, set car_s to the end of the path
          }
          
          // Wrap-up My Car's info 
          MyCar my_car;
          my_car.x = car_x;
          my_car.y = car_y;
          my_car.s = car_s;
          my_car.d = car_d;
          my_car.yaw = car_yaw;
          my_car.speed = car_speed;

          // Wrap-up map_waypoints data with a struct  
          MapWayPoints map_waypoints = {map_waypoints_x, map_waypoints_y, map_waypoints_s, 
                                        map_waypoints_dx, map_waypoints_dy};
          // Wrap-up previous path' data with a struct
          PreviousPath previous_path = {previous_path_x, previous_path_y};            

          // 1. Predict closest cars to get their distance & speed info   
          map<string, CloseCar> predictions = Predict(sensor_fusion, car_s, prev_size, lane);
          
          // 2. Behaviour planning to change target_speed & lane when appropriate
          CarState car_state = Plan(predictions, target_speed, lane);

          // 3. Generate trajectory based on target_speed & lane for car to travel
          auto next_path = Generate_Trajectory(my_car, car_state, map_waypoints, previous_path);

          // Update lane and target_speed;
          lane = car_state.lane;
          target_speed = car_state.target_speed; 

          // END TODO

          msgJson["next_x"] = next_path[0];  // next_x_vals of path
          msgJson["next_y"] = next_path[1];  // next_y_vals of path

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // End "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // End websocket if
  }); // End h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}