#include "plan.h"

const double SPEED_CHANGE = 0.1;       // accelation =  5 m/(s^2) 
const double MAX_SPEED_CHANGE = 0.19;  // accelation < 10 m/(s^2)
const double MAX_TARGET_SPEED = SPEED_LIMIT - BUFFER_V;  // 24.5-2.5 = 22 m/s

const double CLOSE_AHEAD  = 50;        // closest distance to check if need to change lane

const double MAX_COST = std::numeric_limits<const double>::infinity();


CarState Plan(const map<string, CloseCar> &predictions, double target_speed, int lane) {
    /*
    Using predicted data about closest cars to do bahavior planning
    @param predictions,  a list of closest cars
    @param target_speed, previous target_speed set for my car
    @param lane, previous lane set for my car

    @return CarState,  updated state for my car - updated speed & lane if changed
    */
   
    // Retrieve predicted cars
    CloseCar car_ahead  = predictions.find("Ahead")->second;
    CloseCar car_behind = predictions.find("Behind")->second;
    CloseCar car_left_ahead   = predictions.find("Left_Ahead")->second; 
    CloseCar car_left_behind  = predictions.find("Left_Behind")->second;
    CloseCar car_right_ahead  = predictions.find("Right_Ahead")->second;
    CloseCar car_right_behind = predictions.find("Right_Behind")->second;

    double cost_keep, cost_left, cost_right;
    vector<double> total_costs = {};
    
    CarState car_state = {lane, target_speed, false};

    // No car ahead or still far away, just keep lane and speed up as fast as possible
    if ( car_ahead.dist == 9999 ||  car_ahead.dist > CLOSE_AHEAD ) {

       if ( target_speed < MAX_TARGET_SPEED ) { // able to speed up
           target_speed += SPEED_CHANGE;
       }
       car_state.lane = lane;
       car_state.target_speed = target_speed;
        
       return car_state; 
    }

    /* When car_ahead is close - considering 3 options: Keep Lane, Change to Left , Change to Right */

    // 1. Calculate cost for Keep Lane  
    cost_keep = Calculate_cost(car_ahead.dist, car_behind.dist, car_ahead.speed, target_speed);
    total_costs.push_back(cost_keep);    
    //cout << "cost_keep=" << cost_keep << endl;
  
    // 2. Calculate cost for Change to Left lane
    if ( lane > 0 ) {  
        cost_left = Calculate_cost(car_left_ahead.dist, car_left_behind.dist, car_left_ahead.speed, target_speed);      
    }
    else {
        cost_left = MAX_COST;      
    } 
    total_costs.push_back(cost_left);
    //cout << "cost_left=" << cost_left << endl;
   
    // 3. Calculate cost for Change to right lane
    if ( lane < 2 ) {
        cost_right = Calculate_cost(car_right_ahead.dist, car_right_behind.dist, car_right_ahead.speed, target_speed);      
    } else {
        cost_right = MAX_COST;
    }     
    total_costs.push_back(cost_right);                
    //cout << "cost_right=" << cost_right << endl;

    // Find min cost
    vector<double>::iterator best_cost = min_element(begin(total_costs), end(total_costs));
    int best_idx = distance(begin(total_costs), best_cost);

    if (best_idx == 0) {      // keep lane
        car_state.lane = lane;
        car_state.target_speed = Update_target_speed(car_ahead.dist, car_ahead.speed, target_speed);
        car_state.ChangeLane = false;
        //cout << "Decide to Keep Lane.." << endl;
        //cout << "car_state.lane=" << car_state.lane << ", target_speed= " << car_state.target_speed << endl;
    }
    else if (best_idx == 1) { // change to left
        car_state.lane = lane - 1 ;
        car_state.target_speed = Update_target_speed(car_left_ahead.dist, car_left_ahead.speed, target_speed);
        car_state.ChangeLane = true;

        //cout << "Decide to Change to Left.." << endl;
        //cout << "car_state.lane=" << car_state.lane << ", target_speed= " << car_state.target_speed << endl;
    }
    else {                    // change to right
        car_state.lane = lane + 1;
        car_state.target_speed = Update_target_speed(car_right_ahead.dist, car_right_ahead.speed, target_speed);
        car_state.ChangeLane = true;

        //cout << "Decide to Change to Right.." << endl;
        //cout << "car_state.lane=" << car_state.lane << ", target_speed= " << car_state.target_speed << endl;
    }

    return car_state;

} // End Plan


double Update_target_speed(double ahead_dist, double ahead_speed, double target_speed){
    /*
    Update target_speed based on data of car ahead (distance and speed) 
    @param ahead_dist/ahead_speed, data of car ahead 
    @param target_speed, previous target_speed

    @return updated target_speed
    */
   
     // if no car ahead or far away, consider to speed up
    if ( ahead_dist == 9999 || ahead_dist > CLOSE_AHEAD ) {
        if ( target_speed < MAX_TARGET_SPEED ) { // able to speed up
            target_speed += SPEED_CHANGE;
        }
    }
    // faster than car ahead, need to slow down
    else if ( target_speed > ahead_speed ){                   
        if ( (target_speed - ahead_speed) <= MAX_SPEED_CHANGE ){
            target_speed  = ahead_speed;
        } else {
            target_speed -= MAX_SPEED_CHANGE;
        }          
    }         
    return target_speed;
} 