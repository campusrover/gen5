# PA0: Out and Back

## Summary
Write ROS code to drive the robot out 1 meters, rotate 180 degrees, return and rotate 180 degrees to its original orientation. 

The simplest way of doing this is by commanding the robot to go at a certain speed for a certain amount of time.

### Equipment
- TurtleBot3

### Skills you will learn:
- writing a basic ROS node with publisher
- writing a launch file
- simulating a ros program using Gazebo
- installing and running the program on Turtlebot3
  
### ROS ropics
- /cmd_vel

### ROS message types
- [geometry_msgs/Twist.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)

### What to hand in
- source code in a .zip folder
- an informal video of the robot executing the code on Turtlebot3
  
### Tasks
1. write the out_and_back_time.py.
1. publish to `/cmd_vel` at a slow speed (e.g. 0.1 meters per second)
1. write a launch file for your program (see link)
1. test it in simulation (how to measure how far the robot has moved?) (see link)
1. test it on the actual robot (see link)

### Overview
This is a great first project. In it you will apply what you've learned about creating a ROS node and publishing a Topic. In our case we will use the `cmd_vel` topic which commands the robot to move at a certain speeed, either forward/backward (x) or rotate (z) or both at the same time. The `cmd_vel` topic uses the `Twist` message type to represent the desired direction and speed.

The general structure of your program, move_and_back.py, will be something like this: [link to a skeleton of the program]. The skeleton is very well documented to explain what goes where and why it is layed out the way it is.

The key "trick" here is that the way you get the robot to drive a certain distance is to command it to go at a certain speed and then watch the time. In other words, if you know the current time, you can loop until 10 seconds have passed, which would approximate one meter, Do a similar approach to rotate. By the way, be aware of the units of Twist message, linear velocity is in meter/second, and angular velocity is in radians/second.

### Helpful resources
- link to bashrc configuration and ssh troubleshoot
- link to how to get current time
- link to instruction write a launch file
- link to chmod +x the python scripts and `catkin_make`
- link to run ros in simulation

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