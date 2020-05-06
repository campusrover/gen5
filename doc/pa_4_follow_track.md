### Summary

Write ROS code to have the robot follow an indoor track. It will have gradual turns but no sharp corners or intersections. The track can be 20 meters long. The robot is placed at the very end of the track pointing inwards. Behind it there will be a wall so that it doesn't try to drive out the other side. Once the command is given, the robot drives down the track, maintaining as close to the center as possible, and gets all the way to the other end without hitting the wall.

### Equipment

* TB3 + Pi Camera
* Track painted on the floor with tape

### Track
* We're not sure yet. Depending on the programming any of these are possible
* The track will be made out of red blocks, set about 75 cm apart
* The track will be marked by a single (10cm wide) line
* The track will be marked by yellow traffic cones forming a route 75 cm apart

### Skills you will learn:

* Working with and processing Lidar data
* Working with very noisy data, and filtering it to find the signal
* How to use PID processing to make the robot behave reasonably
* Simple state management to control the behavior of the robot

### Content knowledge you will gain:

* /scan topic, its format
* PID Processing, what it means and how it works

### Examples of this in the real world
* supply a link or paper or video)

### What to hand in
* Zip up your source code
* A video (informal) of the robot executing the code

### Tasks

##### Think through the design
* Look at what /scan data looks like using a test program or rqt_chart
* Read about PID processing and consider how you might use it

##### write wall_follow.launch

##### write wall_follow.py
*

##### test it in Rviz

##### test it in the actual robot
