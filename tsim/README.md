# README.md for tsim Package

###  Package description:
* The nuturtle_description package is for HW0 ME495: sensing, Navigation, and Machine Learning for Robotics.
* This package enable turtle move in around a rectangular trajectory. Also show the rqt_plot

### Roslaunch comand
```
$ roslaunch tsim trect.launch
```

### package files:
* config/turtle_rect.yaml: show the parameter of trajectory and turtle rotational and linear speed
* README.md: introduction to package
* msg/PoseError.msg: msgs for turtle pose_error
* launch/trect.launch:complie all files to launch, see more intro info inside of .launch file
* src/turtle_rect.cpp: control turtle move in a rectangle path, and reset when turtle move to the end to the loop of the rectangle.
