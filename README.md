# ME495 Sensing, Navigation, and Machine Learning

Author: Luxi Huang

This README.md contains tasks completeness.



# Tasks submitted:
A.000
A.001
A.002
A.003
A.004
# Tasks completed:

### incomplete homework:
5. B.000
6. B.001
7. B.002
8. B.003
9. B.004          


10. C.000    
11. C.001
12. C.002
13. C.003
14. C.004

## A.000:
#### Fixed Error1:     
$ catkin_link -w2
```
nuturtle_description: warning: package 'std_msgs' should be listed in catkin_package()
 ```
Solution:   In CMakeLists file uncomment line "CATKIN_DEPENDS roscpp rospy std_msgs" on catkin_package()

#### Fixed Error2:
$ catkin_link -w2
```
nuturtle_description: notice: package description starts with boilerplate 'nuturtle_description package is for'
 ```
Solution:rewrite other words in the package.xml file

#### Fixed Error3:
$ roslaunach ...
```
File "/usr/lib/python2.7/subprocess.py", line 394, in __init__
    errread, errwrite)
  File "/usr/lib/python2.7/subprocess.py", line 1047, in _execute_child
    raise child_exception
OSError: [Errno 13] Permission denied
```
wrong solution:add "cat"on launch file:
```
 <param name="robot_description" command="cat '$(find nuturtle_description)/urdf/diff_drive.urdf.xacro'"/>
```
correct solution:
```
<param name="robot_description"
     command="$(find xacro)/xacro $(find nuturtle_description)/urdf/diff_drive.urdf.xacro"/>
  ```
  #### fixed problem 4
  * joint-joint_state_publisher control the wheels in right direction
  > fixed by edit joint and link origin position.

```
catkin_make --only-pkg-with-deps my_package
```
