# pure-pursuit-turtlebot3
ROS Implementation of Pure Pursuit Path Tracking Algorithm on Turtlebot3 Robot.

Turtlebot3 is diff-drive robot, however, example code of pure pursuit [1] is for ackerman steering. 

First, the target point on waypoints is determined by one look ahed distance, then, `move2pose` code [2] is used to control turtlebot3 to the target point.

More detail of algorithm is on original paper [3]

## Requirement
 - Ubuntu 20.04
 - Gazebo 11.0
 - ROS Noetic
 - Turtlebot3
 - gazebo_ros_pkgs (noetic-devel branch)

## RUN
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

rosrun pure-pursuit-turtlebot3 pp-tb3.py
```

## Reference
[1]. https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

[2]. https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py

[3]. https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf