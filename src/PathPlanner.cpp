//
// Created by Mykytiuk Olga, EV-33 on 16.11.18.
//

#include "PathPlanner.h"
#include <vector>
#include <map>
#include "Vehicle.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

PathPlanner::PathPlanner(int current_lane_in, double ref_vel){
 state = PathState::keepLane;
 current_lane = current_lane_in;
 next_lane = current_lane;
 sensor_fusion = sensor_fusion;
}



void PathPlanner::Update() {
    if(state == PathState::keepLane)
        state = KeepLane();
    else if(state == PathState::prepareLaneChangeRight)
        state = PrepareLaneChangeRight();
    else if(state == PathState::prepareLaneChangeLeft)
        state = PrepareLaneChangeLeft();
    else if(state == PathState::laneChangeLeft)
        state = LaneChangeLeft();
    else if (state == PathState::laneChangeRight)
        state = LaneChangeRight();
    else
        state = KeepLane();
}



std::pair<bool, double> PathPlanner::getCarSpeedInfront() {
    bool too_close = false;
    double infrontCarSpeed = speedLimit;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        //check lane of the car
        float d = sensor_fusion[i][6];
        //if car is on my lane
        if ((d < 2 + 4 * egoVehicle.lane + 2) && (d > 2 + 4 * egoVehicle.lane - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) egoVehicle.prev_size * 0.02 * check_speed); //project s value of the car in the future
            if ((check_car_s > egoVehicle.car_s) && (check_car_s - egoVehicle.car_s) < 30) {
                too_close = true;
                return std::make_pair(too_close,check_speed);
            }
        }
    }
    return make_pair(too_close,infrontCarSpeed);
}

PathState PathPlanner::KeepLane(){
    cout << "KeepLane\n";
    //check if some car ahead
    bool too_close = slowDownBeforeCar();
    if (egoVehicle.speed < speedLimit-30) {
        return PathState ::keepLane;
    }
    if (too_close) {
        cout << "too close, searching for faster lane \n";
        int fastestLane = GetFastestLane(current_lane);
        if (fastestLane == current_lane)
            return PathState::keepLane;
        //if 2 lanes away move first to the next one
        if (abs(fastestLane - current_lane) > 1) {
            if (fastestLane > current_lane)
                fastestLane--;
            else
                fastestLane++;
        }
        cout << "NEW FASTEST LANE: " << fastestLane << "\n";
        if (current_lane > fastestLane) {
            next_lane = fastestLane;
            return PathState::prepareLaneChangeLeft;

        } else {
            next_lane = fastestLane;
            return PathState::prepareLaneChangeRight;
        }
    }


}

int PathPlanner::GetFastestLane(const int lane) {
    map<int, vector<double>> laneToVels;
    //check which lane is the best
    for (int i = 0; i < sensor_fusion.size(); i++) {
        //get lane of the car
        //int d = (((double)sensor_fusion[i][6])-2.)/4;
        int d = sensor_fusion[i][6];
        int d_lane = 0;
        for(int l = 0; l < 3; l++) {
            if((d<2+4*l +2) && (d >2+4*l-2)) {
                d_lane = l;
                break;
            }
        }
        if (d_lane > 2) {
            //std::cout << "AAAAAAAAAAAAAA lane: " << d_lane << "\n";
        }
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double car_speed = sqrt(vx*vx + vy*vy);
        if ( laneToVels.find(d_lane) == laneToVels.end() ) {
            // not found
            vector<double> speedVector {car_speed};
            laneToVels.insert(make_pair(d_lane, speedVector));
        } else {
            // found
            auto speedVector = laneToVels.at(d_lane);
            speedVector.push_back(car_speed);
        }
    }
    //get the fastest lane
    int fastest_lane = lane;
    int fastest_velocity = 0;
    for (int i = 0; i < 3; i ++) {
        if (laneToVels.find(i) == laneToVels.end() ) {
            fastest_lane = i;
            std::cout << "fastest lane is empty! : " << fastest_lane << "\n";
            break;
        } else {
            auto speedVector = laneToVels.at(i);
            double average = accumulate( speedVector.begin(), speedVector.end(), 0.0)/speedVector.size();
            if (average >= fastest_velocity) {
                fastest_lane = i;
                fastest_velocity = average;
            }
        }

    }
    return fastest_lane;
}

PathState PathPlanner::PrepareLaneChangeLeft() {
    cout << "PrepareLaneChangeLeft\n";
    changeLeftStepsCnt ++;
    cout << "changeLeftStepsCnt got : " << changeLeftStepsCnt << "\n";
    slowDownBeforeCar();

    bool canChange = CanChangeLane();
    if (canChange) {
        changeLeftStepsCnt = 0;
        return PathState::laneChangeLeft;
    }
    if (changeRightStepsCnt > maxStepsChangeLeft) {
            cout << "changeLeftStepsCnt achived : " << changeLeftStepsCnt << "\n";
            changeLeftStepsCnt = 0;
            return PathState::keepLane;

    }
    return PathState::prepareLaneChangeLeft;
}

bool PathPlanner::slowDownBeforeCar() {
    auto infrontCar = getCarSpeedInfront();
    bool too_close = infrontCar.first;
    double infrontCarSpeed = infrontCar.second;
    if (too_close && (egoVehicle.speed > (infrontCarSpeed - 5))) {
        cout << "Getting slower, car infront\n";
        egoVehicle.speed -=0.224; //5m/sec
    } else if(egoVehicle.speed < speedLimit) {
        egoVehicle.speed +=0.224;
    }
    return too_close;
}

bool PathPlanner::CanChangeLane() {
    Vehicle* vehicleBehind = getNextLaneVehicleBehind(next_lane);
    bool clearBehind;
    if (vehicleBehind != nullptr) {
        clearBehind = checkVehicleCollide(vehicleBehind);
    } else {
        cout << "No vehicle behind\n";
        clearBehind = true;
    }
    cout << "clearBehind " << clearBehind << "\n";
    bool clearAhead;
    Vehicle* vehicleAhead = getNextLaneVehicleAhead(next_lane);
    if (vehicleAhead != nullptr) {
        clearAhead = checkVehicleCollide(vehicleAhead);
    } else {
        clearAhead = true;
        cout << "No vehicle ahead\n";
    }
    cout << "clearAhead " << clearAhead << "\n";
    bool canChange = clearBehind && clearAhead;

    if (canChange) {
        cout << "CAN CHANGE TO!!!" << next_lane<<"\n";
    } else {
        cout << "CAN NOT CHANGE TO !!!" << next_lane<<"\n";
    }
    return canChange;
}

bool PathPlanner::checkVehicleCollide(Vehicle * vehicle) {
    double vx = vehicle->vx;
    double vy = vehicle->vy;

    double prediction_time = 0.1; //50 controller cycles 0.02
    double speed = sqrt(vx*vx + vy*vy)/0.44704;///0.44704;
    cout << "other speed: " << speed << "\n";
    double s_now = vehicle->s;
    cout << "vehicle next s: " << s_now << "\n";
    cout << "my s: " << egoVehicle.car_s << "\n";
    cout << "vehicle next d: " << vehicle->d << "\n";
    cout << "my d: " << egoVehicle.d << "\n";
    double s =s_now + ((double)egoVehicle.prev_size*prediction_time*speed); // prediction_time //0.02
    //double ego_speed = sqrt(egoVehicle.vx*egoVehicle.vx + egoVehicle.vy*egoVehicle.vy);
    double ego_s = egoVehicle.car_s + ((double)egoVehicle.prev_size*prediction_time*egoVehicle.speed);
    cout << "egoVehicle.speed: " << egoVehicle.speed << "\n";
    cout << "checkVehicleCollide meters: " << abs(ego_s-s) << "\n";
    return ( (abs(ego_s-s)/egoVehicle.speed)  > secToCollide) && (abs(s_now-egoVehicle.car_s)) > metersBetweenCars && egoVehicle.speed > 30;
}

Vehicle* PathPlanner::getNextLaneVehicleAhead(int lane) {
    vector<Vehicle> vehiclesAhead;

    for (int i = 0; i < sensor_fusion.size(); i++) {
        //check lane of the car
        float d = sensor_fusion[i][6];
        float s = sensor_fusion[i][5];
        //if car is on the fastest lane
        if ((d < 2 + 4 * lane + 2) && (d > 2 + 4 * lane - 2) && s > egoVehicle.s ) {
            Vehicle vehicle = Vehicle(sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3],
                                      sensor_fusion[i][4], sensor_fusion[i][5], sensor_fusion[i][6]);
            vehiclesAhead.push_back(vehicle);
        }
    }
    std::sort(vehiclesAhead.begin(), vehiclesAhead.end(),
                  [](Vehicle const &a, Vehicle const &b) {
                      return a.s > b.s;
                  });

    if (!vehiclesAhead.empty())
        return &vehiclesAhead.at(0);
    else return nullptr;
}

Vehicle* PathPlanner::getNextLaneVehicleBehind(int lane) {
    vector<Vehicle> vehiclesAhead;

    for (int i = 0; i < sensor_fusion.size(); i++) {
        //check lane of the car
        float d = sensor_fusion[i][6];
        float s = sensor_fusion[i][5];
        //if car is on the fastest lane
        if ((d < 2 + 4 * lane + 2) && (d > 2 + 4 * lane - 2) && s <= egoVehicle.s ) {
            Vehicle vehicle = Vehicle(sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3],
                                      sensor_fusion[i][4], sensor_fusion[i][5], sensor_fusion[i][6]);
            vehiclesAhead.push_back(vehicle);
        }
    }
    std::sort(vehiclesAhead.begin(), vehiclesAhead.end(),
              [](Vehicle const &a, Vehicle const &b) {
                  return a.s < b.s;
              });

    if (!vehiclesAhead.empty())
        return &vehiclesAhead.at(0);
    else return nullptr;
}



PathState PathPlanner::PrepareLaneChangeRight(){
    cout << "PrepareLaneChangeRight\n";
    slowDownBeforeCar();

    bool canChange = CanChangeLane();
    if (canChange) {
        changeRightStepsCnt = 0;
        return PathState::laneChangeRight;
    }
    else {
        if (changeRightStepsCnt < maxStepsChangeRight) {
            changeRightStepsCnt ++;
            cout << "changeRightStepsCnt: " << changeRightStepsCnt << "\n";
            return PathState::prepareLaneChangeRight;
        } else {
            changeRightStepsCnt = 0;
            return PathState::keepLane;
        }
    }
}

PathState PathPlanner::LaneChangeRight() {
    cout << "LaneChangeRight\n";
    egoVehicle.lane = next_lane;
    current_lane = next_lane;
    return PathState ::keepLane;
}

PathState PathPlanner::LaneChangeLeft(){
    cout << "LaneChangeLeft\n";
    egoVehicle.lane = next_lane;
    current_lane = next_lane;
    return PathState ::keepLane;
}

