#include "prediction.h"

map<string, CloseCar> Predict(const vector<vector<double>> &sensor_fusion, double car_s, int prev_size, int lane){

    /*
    Using sensor fusion data to predict other cars' positions and get closest cars
    @param sensor_fustion, other car's position and speed info
    @param car_s, my car's future s
    @param prev_size, previous path length
    @param lane, my car's lane

    @return a list of closest cars - total of 2*3 = 6 cars ( one ahead and one behind for each lane)
    */
    // Set distance metrics to check cars
    double min_s = 9999;
    double max_s = -1; 

    double min_left_s  = 9999;
    double max_left_s  = -1;

    double min_right_s = 9999;   
    double max_right_s = -1;

    // Cars that are closest to my car 
    // Initilize with very large distance value 
    CloseCar car_ahead = {9999,0};
    CloseCar car_behind = {9999,0};
    CloseCar car_left_ahead   = {9999,0};
    CloseCar car_left_behind  = {9999,0};
    CloseCar car_right_ahead  = {9999,0};
    CloseCar car_right_behind = {9999,0};
   
    // Check each car's status
    for(int i = 0; i < sensor_fusion.size(); i++){
        
        // retrieve position & speed of checked car 
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];
        float d = sensor_fusion[i][6];

        // predict future s of checked car
        check_car_s += ((double) prev_size * 0.02 * check_speed);
        // distance to my car
        double dist = check_car_s - car_s;

        // if checked car in same lane as mine
        if ( d > (2+4*lane-2) && d < (2+4*lane+2) ){              
            // if car is ahead and closest to my car
            if ( dist > 0 &&  check_car_s < min_s){
                car_ahead.dist  = dist;
                car_ahead.speed = check_speed;
                min_s = check_car_s;
            } 
            // car is behind and closed to my car
            else if (dist < 0 && check_car_s > max_s) { 
                car_behind.dist  = abs(dist);
                car_behind.speed = check_speed;
                max_s = check_car_s;
            }   
        } 
        // if checked car in left lane to my car
        else if ( lane > 0 && d > (4*lane-4) && d < (4*lane) ) {
            // if car is ahead and closest to my car
            if ( dist > 0 &&  check_car_s < min_left_s){
                car_left_ahead.dist  = dist;
                car_left_ahead.speed = check_speed;
                min_left_s = check_car_s;
            } 
            // car is behind and closed to my car
            else if (dist < 0 && check_car_s > max_left_s) { 
                car_left_behind.dist  = abs(dist);
                car_left_behind.speed = check_speed;
                max_left_s = check_car_s;
            }   
        } 
        // if checked car in right lane to my car
        else {
            // if car is ahead and closest to my car
            if ( dist > 0 &&  check_car_s < min_right_s){
                car_right_ahead.dist  = dist;
                car_right_ahead.speed = check_speed;
                min_right_s = check_car_s;
            } 
            // car is behind and closed to my car
            else if (dist < 0 && check_car_s > max_right_s) { 
                car_right_behind.dist  = abs(dist);
                car_right_behind.speed = check_speed;
                max_right_s = check_car_s;
            }   
        } // End if
         
    }// End for sensor_fusion

    map<string, CloseCar> predictions;

    predictions["Ahead"]  = car_ahead;
    predictions["Behind"] = car_behind;
    predictions["Left_Ahead"]  = car_left_ahead;
    predictions["Left_Behind"] = car_left_behind;
    predictions["Right_Ahead"] = car_right_ahead;
    predictions["Right_Behind"] = car_right_behind;

    return predictions;

}// End Predict


