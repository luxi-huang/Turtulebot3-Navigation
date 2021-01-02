# README.md for tsim Package
- Author：Luxi Huang
- Skills: ROS (Node, Rviz), C++, Path-Planning, navigation. 
---

##  Package description:
* This package control turtle move in  a rectangular trajectory and user-specified waypoints. 
* Predict turtle position, and draw `rqt_plot` of absolute pose error (x,y,theta). 
* It has a `traj_reset` service can be called to reset turtle to left corner of rectangular. 

## Trect example: 
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/master/img/rect.gif?raw=true" alt="drawing" /> </p>  

## turtle_odom.launch example:
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/review/img/turtle_way_combine.gif?raw=true" alt="drawing" /> </p>  


## Package files:
### 1. `src/turtle_rect_node.cpp`:
- This file initial `NodeHandle` and create `turtle_rect` node. It includes `TurtleRect` class to make turtle move in  a rectangular trajectory infinitely.
-  It predict turtle position, and draw `rqt_plot` of absolute pose error (x,y,theta).
-  Turtle can be reset to left corner of rectangular by call `traj_reset` service. 

### 2. `src/turtle_rect.cpp`:
- This is the Class Constructor for `TurtleRect`.

### 3. `src/turtle_way.cpp`:
- This file create node called `turtle_way` that has turtle follow a trajectory of user-specified waypoints, it implement with `rigid2d` library. 

### 3. `include/turtle_rect.cpp`:
- Header file for the `TurtleRect` class.

### 4. `launch/trect.launch` :
-  This launch file runs a window to shows a turtle move in  a rectangular trajectory, and draw `rqt_graph` of show turtle pose error. It launch following files:
   - `turtle_rect` node 
   - `turtlelism` node 
   - `turtle_rect.yaml` file 
   - `rqt_plot` node

### 5. `config/turtle_rect.yaml` :
   - It contains following robot path control parameters:
   - `x`: The x coordinate of the lower left corner of a rectangle
   - `y`: The y coordinate of the lower left corner of a rectangle
   - `width`: The width of the rectangle
   - `height`: The height of the rectangle
   - `trans_vel`: The translational velocity of the robot
   - `rot_vel`: The rotational velocity of the robot
   - `frequency`: The frequency of the control loop

### 6. `launch/turtle_pent.launch` :
   -  This launch file runs a window to shows a turtle move in  a pent trajectory, and draw `rqt_graph` of show turtle pose error. It launch following files:
   - `turtle_way` node 
   - `turtlelism` node 
   - `turtle_way.yaml` file 
   - `rqt_plot` node

### 7. `launch/turtle_odom.launch`
   - This launch file enable robot model move in rviz with turtleway trajectory.
   - `diff_drive.urdf.xacro`: load robot model 
   - `rviz`: load model on rviz
   - `turtle_pent.launch` : load launch file to control turtle to move in pent trajectory and publish velocity 
   - `fake_encoder` node: subscribe robot velocity and publish joint_state for each wheels 
   - `odometer` node: subscribe joint state and publish robot odometer and tf. 

## How to launch Files
```
$ roslaunch tsim trect.launch --ros-args
```
Optional Arguments:
   - `plot_gui` （default "false"） : set value to true to draw rqt_plot of robot pose error (x, y, theta).
     - `roslaunch tsim trect.launch plot_gui：=true`

## Result rqt_plot for turtle_rect :

The below chart is the error between predict turtle pose and real turtle pose. 
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/master/img/turtleRect_errorPlot.png?raw=true" alt="drawing" /> </p>  

## Result rqt_plot for turtle_way:

The below chart is the error between predict turtle pose and real turtle pose. 
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/review/img/turtle_way_error.png?raw=true" alt="drawing" /> </p>  