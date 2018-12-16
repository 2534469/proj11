//
// Created by Mykytiuk Olga, EV-33 on 16.11.18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <stdio.h>
#include <vector>
#include "Vehicle.h"
#include <math.h>

using namespace std;


enum class PathState {
    keepLane = 0,
    prepareLaneChangeLeft = 1,
    prepareLaneChangeRight = 2,
    laneChangeLeft = 3,
    laneChangeRight = 4
};

class PathPlanner {
public:
    PathState state = PathState::keepLane; //at the beginning
    int current_lane;
    int next_lane;
    vector<vector<double>> sensor_fusion;
    Vehicle egoVehicle;

    PathPlanner(int current_lane_in, double ref_vel);

    void SetSensorFusion(const vector<vector<double>> &sensor_fusion_in){
        sensor_fusion = sensor_fusion_in;
    }

    void Update();

    void SetEgoVehicle(Vehicle & _egoVehicle) {
        egoVehicle = _egoVehicle;
    }
    int GetFastestLane(const int lane);
    bool CanChangeLane();
    pair<bool, double> getCarSpeedInfront();

private:
    Vehicle* getNextLaneVehicleAhead(int lane);
    Vehicle* getNextLaneVehicleBehind(int lane);

    bool checkVehicleCollide(Vehicle * vehicle);

    double const speedLimit = 49.5;

    //max number of steps in PrepareChangeLeft
    int const maxStepsChangeLeft = 50;
    int const maxStepsChangeRight = 50;
    int const metersBetweenCars = 80;
    double const secToCollide = 3;
    //stepsCounter in PrepareChangeLeft
    int changeRightStepsCnt = 0;
    int changeLeftStepsCnt = 0;



    PathState KeepLane();

    PathState PrepareLaneChangeLeft();

    PathState PrepareLaneChangeRight();

    PathState LaneChangeRight();

    PathState LaneChangeLeft();

    bool slowDownBeforeCar();

    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
};


#endif //PATH_PLANNING_PATHPLANNER_H
