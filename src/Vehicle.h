//
// Created by Mykytiuk Olga, EV-33 on 16.11.18.
//

#ifndef PATH_PLANNING_EGOVEHICLE_H
#define PATH_PLANNING_EGOVEHICLE_H


class Vehicle {

public:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
    int lane;
    double car_s;
    int prev_size = 0;

    Vehicle(double speed_in, int lane_in, double car_s_in, int prev_size_in):
    speed(speed_in),lane(lane_in), car_s (car_s_in), prev_size(prev_size_in) {};
    Vehicle ():speed(0),lane(1), car_s(0)  {};

    Vehicle(double x_in, double y_in, double vx_in, double vy_in, double s_in, double d_in):
    x(x_in),y(y_in), vx(vx_in), vy(vy_in), s(s_in), d(d_in){};

};


#endif //PATH_PLANNING_EGOVEHICLE_H
