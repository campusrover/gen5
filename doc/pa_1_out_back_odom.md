# PA1: Out and Back

## Summary
Write ROS code to drive the robot out 1 meters, rotate 180 degrees, return and rotate 180 degrees to its original orientation. 

We use a slightly more sophisticated approach. The robot reports what position and orientation (pose) it (thinks it) is. We track it and stop the motion or rotation accordingly.

### Equipment
- TurtleBot3

### Skills you will learn:
- writing a basic ROS node with both publisher and subscriber
- writing a launch file with multiple nodes
- designing and writing ROS action
- invoking action using an action client
- writing a ros callback function in python
  
### ROS ropics
- /cmd_vel
- /odom

### ROS message types
- [geometry_msgs/Twist.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)
- [geometry_msgs/Pose.msg](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html)
- [nav_msgs/Odometry.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)

### What to hand in
- source code in a .zip folder
- an informal video of the robot executing the code on turtlebot3
  
### Tasks
- write the out_and_back_odom.py
- write rotate_server.py
- write run_server.py
- publish to `/cmd_vel` at a slow speed (e.g. 0.1 meters per second)
- write a launch file for your program
- test it in simulation
- test it on the actual robot

### Hints
- keep publishing to `/cmd_vel` until the robot thinks it has moved one meter or rotated 180 degrees using its odometry data
- be aware of the units of Twist message, linear velocity is in meter/second, and angular velocity is in radians/second
- be sure to modify `CMakeList.txt` to include the actions you have created
  

### Helpful resources
- link to bashrc configuration and ssh troubleshoot
- link to instruction write a launch file
- link to chmod +x the python scripts and `catkin_make`
- link to run ros in simulation
