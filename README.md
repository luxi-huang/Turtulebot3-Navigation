# ME495 Sensing, Navigation, and Machine Learning

Author: Luxi Huang

This README.md contains tasks completeness. 



# Tasks submitted:
A.000git
# Tasks completed:

### All homework:

1. A.001
2. A.002
3. A.003
4. A.004


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
