#ifndef CAR_H
#define  CAR_H

// struct to wrap-up data for passing parameters

// My car's info
struct MyCar
{
    /* data */
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

};

// Closest car 
struct CloseCar
{
    double dist;   // distance between the other car and my car
    double speed;  // speed of the car
};

// My car's state for planning 
struct CarState
{
    int lane;
    double target_speed;

    bool ChangeLane;
};

#endif // CAR_H