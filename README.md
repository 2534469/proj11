# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Model Documentation

For the project I implemented a state machine with following states:
* KeepLane
* PrepareLaneChangeRight
* PrepareLaneChangeLeft
* LaneChangeLeft
* LaneChangeRight

According to my model, the ego-car stays in the same line(KeepLane), until 
a car in front is detected.

If a car is detected, the fastest lane gets calculated.
In my case it's a lane, where the average speed between all cars is the lowest or the one which is empty.
If the empty line is not the one next to the current lane, then the middle lane becomes a target first.
The state changes to the PrepareLaneChangeRight or PrepareLaneChangeLeft depending on the fastest lane.
These 2 States have a counter, if they last too long, they change back to the KeepLane state.
In these state I check if there is a safe possibility to change 
the lane, by calculating the distance between the ego-car and 2 cars behind and in front on the target lane.
Both the current distance between the cars and the distance calculated for the future is compared with the threshold.
I also check time to collision, i.e. the distance to the rear/front car divided by the current speed.
 This code you can find in the PathPlanner.cpp in the 
 ```cpp
 bool PathPlanner::CanChangeLane()
 ```
 For the points prediction I use spline library, as explained in the project video.
 