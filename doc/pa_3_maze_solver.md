# PA2: Wall Follow

## Summary
A maze is provided built out of the red blocks. The robot is placed somewhere in the middle of the maze and it drives, following walls, identifying corners, to eventually find its way out of the maze. All the corners in the maze are 90 degrees, and there are no loops. The whole maze will be approximately 3x3 meters, and the passageways will be approximately 0.5-0.75 meters.

Write a python script to control a robot running a maze. From anywhere in the maze, the robot needs to find a way out, that is to the exit. You will know the exit because there are no obstacles within 1 meter in all directions. 

"Challenge by Choice": You may find that the /odom topic has some useful information to help solve this problem. Odometery is not required to solve this problem, but it could lead to a more robust solution if used correctly. 

You should write and test your code first on the simulator. This command will give you a pretty nice "built-in" sort-of-maze using Gazebo

Remember it's worth trying to work in the simulator first. If your code works well in the simulator you know that your algorithm and logic are right. However, even though it works perfectly in the simulator, it might encounter unexpected problems in the real world. The physical robot will behave a lot differently than the simulated one. Take that into account.

The wall-following algorithm can be the basis of the solution, but it is not the only one. And even if you use that, going around corners or handling dead ends will need special attention. But you are encouraged to try different algorithms. You can find tons of algorithms online. Feel free to use any of them. You can invent your own algorithm as well.

You are free to refer to hints for inspiration, and are encouraged to experiment with novel ideas.

### Equipment
- TurtleBot3
- Wall made out of red blocks

### Skills you will learn:
- working with and processing Lidar data
- processing with and filtering sensor noise
- using math (especially geometry) to solve real world robotics problems
- construct a virtual world in gazebo
  
### ROS ropics
- /scan
- /cmd_vel
- /odom

### ROS message types
- [sensor_msgs/LaserScan Message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)
- [geometry_msgs/Twist.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html)

### Both team members submit the same:

1. The source code, including a readme explaining what it is, how to run it and any other reflections. This should be a nice ROS package directory structure as before.
1. A video of the real robot running the maze successfully. This can be totally rough, unedited, with crazy laughing in the background. We just want to see that it worked.

### Each teammember submits their own:

1. Design note + personal reflection: topics such as: Problems you had in getting the program to work; What more you would do if you had more time; how working with a teammate felt; how effective it was
  
### Tasks
- write the maze_runner.py
- write a launch file for your program
- build a maze in gazebo
- test the algorithm in the virtual maze
- test it on the actual robot

### The Maze
There is a maze already set up next to the lab for you to practice on. You can change it if you want, but it doesnt need to be too complex.
Each team can create their own maze as long as it is simuilarly complex as the one in this diagram. 
  
**NB**: When creating a maze, make sure that all of the walls are connected and has no cycles. If you were to draw your maze, you should be able to draw all lines without lifting your pen (you can trace over lines twice). This is important if you choose a wall following algorithm - the robot might get stuck in circles going around a disconnected wall!

### Helpful resources
- link to bashrc configuration and ssh troubleshoot
- link to instruction write a launch file
- link to chmod +x the python scripts and `catkin_make`
- There are multiple turtlebot3_stages in Gazebo. Stage 4 is good to play with, but don't be afraid to try some other stages too. Once you think you have a good wall follower, see if it can do a lap around the outer wall of stage 4 (in the left hand menu of gazebo, use the pose fields under the turtlebot to alter it's initial position)

### Process
This is a pair programming assignment. You've been assigned a teammate. It's fine to talk to everyone else, as well as google for ideas. Just  as long as the code is written and fully understood by both members of the team. You will submit one solution for each team (see below.)

Since this project requires collaberation, you may wish to start using Github to share code. Is it your first time using Git and Github? [Try this extremely basic guide to using Github](https://guides.github.com/activities/hello-world/) [And this is a handy guide to Git terminal commands](http://rogerdudler.github.io/git-guide/)


### Hints
- beware of invalid data from the lidar. the lidar on our robots has particular limitations: The specs say that minimum distance is 120 mm (12 cm, about 5 inches) and maximum distance is 3.5 meters.
- Break the problem down into smaller parts. Think about situations that robot may find itself in, and how it should behave given that circumstance.
- Wall following is tricky because depending on how you do it, it might oscilate back and forth. Read up about PID below, it has some good but advanced ideas.
- Note that as you are following a wall, you need to notice when the wall suddenly disappears and at that point you have to turn to keep following it.
- Conversely the robot might be in the middle where all walls are far and as far from it, in which case you have to just drive until you get to the wall and then start following it
- Feel free to google for wall following robot and be inspired by what you read. Just don't copy the code, please.
- Here are some solutions that might be helpful, but please do not limit to these.
  
### Right hand Rule
[https://en.wikipedia.org/wiki/Maze_solving_algorithm](https://en.wikipedia.org/wiki/Maze_solving_algorithm) - For this assignment, you can choose either one from Right-Hand Rule (which means you will turn to the rightmost route whenever possible) and Left-hand Rule (which means you will turn left whenever possible). Here are some examples for what you should expect when using Right-hand Rule:

### Working with LaserScan data
Here is a great video which provides almost everything you need to understand to manipulate the LaserScan data: [Reading Laserscan Data](http://www.theconstructsim.com/read-laserscan-data/) The most important field that you need to understand is the "ranges". You can access the reading of a specific angle by using "range[index]", and access a subarray by using "range[index0:index1]"

An infinite LIDAR value in a gazebo similation is inf (infinity), while that on the actual Turtlebot3 is 0. A 0 reading from Turltebot3 can refer to either infinity or actual 0 (obstacle is right next to Turtlebot3). Check the valid ranges and also remember that the Lidar's range is not infinite!

###  PID Control
PID is a well known technique for smoothing out control, to avoid oscilation. Applying PID control to your algorithm will make it more robust and perform more consistently. PID control is a smoothing function which helps you better follow the wall without too much fluctuation. It will make the robot turn at a higher speed when you are too away from the designated route (too far or too close to the wall), and turn at a slower speed when you are close to it.

Here are some great links. You will be amazed at how well this works.

* [MIT Pid Video](https://www.youtube.com/watch?v=4Y7zG48uHRo)
* [ROS PID Package ](http://wiki.ros.org/pid)
* [PID Control from Lets Make RObots](https://www.robotshop.com/letsmakerobots/pid-control)


