#include "cost.h"

// Calculate total costs
double Calculate_cost(double dist_ahead, double dist_behind, double ahead_speed, double my_speed){
    /*
    @param dist_ahead/dist_behind,  distance between car ahead & behind  to my car
    @param ahead_speed/my_speed,   speed of car ahead & my car

    @return total costs
    */
    double cost_dist = std::max(Calc_distance_cost(dist_ahead) , Calc_distance_cost(dist_behind));
    double cost_speed = Calc_speed_cost(ahead_speed, my_speed);

    double total_cost = W_DISTANCE * cost_dist + W_SPEED * cost_speed;

    return total_cost;

} // End function

// Calculate cost related to distance
double Calc_distance_cost(double dist){
    double cost;
    
    if (dist < BUFFER_DIST) { // safety buffer
        cost = 35 - dist;
    } 
    else {
        cost = (1.0/dist);
    }
    return cost;

} // End function

// Calculate cost related to speed 
double Calc_speed_cost(double ahead_speed, double my_speed){
    double max_target_speed = SPEED_LIMIT - BUFFER_V;
    double cost;

    if ( ahead_speed >= my_speed ) {
        cost = (max_target_speed - my_speed) / max_target_speed;
    } else  {
        cost = my_speed - ahead_speed ;
    } 
    return cost;
}

/*
double Calc_efficiency_cost(double target_speed){
    
    double max_target_speed = SPEED_LIMIT - BUFFER_V;
    double cost;
    if (target_speed < max_target_speed) {
        cost = (max_target_speed - target_speed) / max_target_speed;
    }
    else if (target_speed < SPEED_LIMIT){
        cost = (target_speed - max_target_speed) / BUFFER_V; 
    } else {
        cost = 1;
    }
    return cost;

}// End function
*/

