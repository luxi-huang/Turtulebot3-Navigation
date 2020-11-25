# README.md for nuturtle_description Package
- Author: Luxi Huang
- Skills: ROS (Rviz,URDF) 

---

##  Package Description:
* This package creates diff_drive robot with caster and load on Rviz.
* Load robot parameters from yaml file.
* It implement joint_state_publisher to control robot wheels.
 
## 
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/master/img/robot_descreption.png?raw=true" alt="drawing" /> </p>  

## Package Files:

### 1. `urdf/diff_drive.urdf.xacro`:
  - Build diff_drive robot model in URDF description.
  - Load `diff_drive.macro.xacro` file and `diff_params.yaml` file.  

### 2. `urdf/diff_drive_macro.xacro`:
  - Contains macro to build robot for modularity and concisely   

### 3. `config/diff_params.yaml`:
  - contains robot parameters.
  - `wheel_width`: The width of the wheels
  - `wheel_radius`: The radius of the wheels
  - `wheel_base`: The distance between the centers wheels
  - `chassis_length`: The length of the main chassis link
  - `chassis_thickness`: The thickness of the plate that forms the chassis
  - `wheel_axel_offset`: The distance from the bottom of the chassis.  

### 4. `launch/view_diff_drive.launch`
  - launch following files:
    - `diff_drive.urdf.xacro` file 
    - `joint_state_publisher` plug_in  
    - `robot_state_publisher` plug_in 

### 5. `config/config.rviz`

* saved view of robot that displaying on rivz.

## Launch Files 
```
$ roslaunch nuturtle_description view_diff_drive.launch --ros-args
```
Optional Arguments:
  - `use_jsp_gui` (default "true"):  Launch the joint_state_publisher gui to publish joint angles
    - `roslaunch nuturtle_description view_diff_drive.launch use_jsp_gui:=true`
