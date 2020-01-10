# README.md for nuturtle_description Package

###  Package description:
* The nuturtle_description package is for HW0 ME495: sensing, Navigation, and Machine Learning for Robotics.
* This package can load diff_drive file on rivz. Also joint_state_publisher can played on Rviz.

### Roslaunch comand
```
$ roslaunch nuturtle_description view_diff_drive.launch
```

### package files:
* urdf/diff_drive.urdf.xacro:contains diff_drive robot model
* README.md: introduction to package
* config/diff_params.yaml: contains robot parameters.
* launch/view_diff_drive.launch:complie all files to launch, see more intro info inside of .launch file
* config/config.rviz: configuration of rviz file for diff_drive robot.
