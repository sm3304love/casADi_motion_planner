# nmpc_motion_planner
This repository includes Nomlinear model predictive control based motion planner algotrithm for UR20 based CasADi and ROS

## Dependancies
* [UR20 gazebo simulation environment](https://github.com/sm3304love/ur20_description)
* ROS Noetic
* [CasADi](https://github.com/casadi/casadi)

## How to use
1. Launch gazebo simulation
```
roslaunch ur20_description velocity_sim.launch
```
2. Run NMPC motion planner
```
rosrun nmpc_motion_planner nmpc_planner
```

## Visual
https://github.com/sm3304love/nmpc_motion_planner/assets/57741032/af37be73-35ff-477f-a222-de8c3f7b43c2

## Modifying NMPC Parameters
You can change the weight parameters within the nmpc_prob.cpp file.
![Screenshot from 2024-03-22 14-43-01](https://github.com/sm3304love/nmpc_motion_planner/assets/57741032/f8c9a3a1-2def-4376-9839-18e1be8475f5)
