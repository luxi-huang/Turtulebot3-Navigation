# README.md for nuturtle_description Package
- Author: Luxi Huang
- Skills: ROS (Rviz, xacro, macro.xacro, yaml) 

---

##  Package Description:
* This package creates diff_drive robot with caster and load on Rviz.
* Load robot parameters from yaml file.
* It implement joint_state_publisher to control robot wheels.
 
## 
 <!-- <p align="middle"> <img src="https://github.com/luxi-huang/portfolio/blob/master/img/posts/Whisker/original_graph.png?raw=true" alt="drawing" height="600"/> </p>   -->

## Package Files:

### `urdf/diff_drive.urdf.xacro`:
  - Build diff_drive robot model in URDF description.
  - Load `diff_drive.macro.xacro` file and `diff_params.yaml` file.  

### `urdf/diff_drive_macro.xacro`:
  - Contains macro to build robot for modularity and concisely   

### `config/diff_params.yaml`:
  - contains robot parameters.
  - `wheel_width`: The width of the wheels
  - `wheel_radius`: The radius of the wheels
  - `wheel_base`: The distance between the centers wheels
  - `chassis_length`: The length of the main chassis link
  - `chassis_thickness`: The thickness of the plate that forms the chassis
  - `wheel_axel_offset`: The distance from the bottom of the chassis.  

### `launch/view_diff_drive.launch`
  - launch following files:
    - `diff_drive.urdf.xacro` file 
    - `joint_state_publisher` plug_in  
    - `robot_state_publisher` plug_in 

### `config/config.rviz`

* saved view of robot that displaying on rivz.



