# PA2: Wall Follow

## Summary
Write ROS code that allows Turtlebot3 to follow a wall or parameter. 

You are free to refer to hints for inspiration, and are encouraged to experiment with novel ideas.

### Equipment
- TurtleBot3
- Wall made out of red blocks

### Skills you will learn:
- working with and processing Lidar data
- processing with and filtering sensor noise
- using math (especially geometry) to solve real world robotics problems
  
### ROS ropics
- /scan
- /cmd_vel

### ROS message types
- [sensor_msgs/LaserScan Message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)
- [geometry_msgs/Twist.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)

### What to hand in
- source code in a .zip folder
- an informal video of the robot following the wall
  
### Tasks
- write the wall_follower.py
- write a launch file for your program
- test it on the actual robot

### Hints
- the robot should be able to detect which side the wall is located
- the robot should always aim to run at some fixed distance in parallel with the wall
- consider when the robot would need to turn, there are multiple cases.
- beware of invalid data from the lidar
  

### Helpful resources
- link to bashrc configuration and ssh troubleshoot
- link to instruction write a launch file
- link to chmod +x the python scripts and `catkin_make`