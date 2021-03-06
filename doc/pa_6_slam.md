

#### Introduction

In robotics, SLAM (simultaneous localization and mapping) is a powerful algorithm for creating a map which can be used for autonomous navigation. When working with SLAM on the Turtlebot3, the `turtlebot3_slam` package provides a good starting point for creating a map. Although this package does provide preconfigured launch files for using SLAM, there are parameters that can be tuned in order to improve performance. This guide will explain how to tune these parameters and how to use SLAM on the Turtlebot3. This homework will give you your first experience with SLAM on the Turtlebot3

#### Assignment

1. Each student should do this individually, although it is fine to consult with each other if you are having problems.
2. I would like you to select an indoor place with a smooth surface to map. It should be bigger than a single office. For example it could be a quiet hallway, a space in the basement. I can help you find somewhere if you have trouble. Not too big, because the robot moves slowly. And not featureless because AMCL requires landmarks.
3. Using a TB3, and following the hints and instructions here [Robotis Turtlebot3 Slam](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) and here [Robotis Turtlebot3 Navigation](http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation) create a map of that area, save it to disk and then demonstrate the robot navigating in a few scenarios.
4. Keep in mind the limitations of LIDAR - the minimum and maximum distances

#### Submission

This will not require any coding. You should submit to Latte a video demonstrating success and a short lab report explaining what you had to do to get it to work.

1. I will be looking at how smoothly or not the navigation is in your video. If you havent studied on how to tune slam the robot may get confused or get stuck or go around in circles. Smoothness counds
1. I will be looking at home complicated your space is. Unfortunately everyone is copying Brad's original maze outside our lab. This is taking the easy way out. I would love to see some more challenging places for your drive.
1. Make sure the space has some narrow parts and I would like to see the robot go through the narrow part during navigation because that's a little more difficult.
1. This is an individual assignment. It's fine to discuss and brainstorm and learn from each others experiences. But in the end I need everyone to have done it individually.
1. Dont forget the lab report writeup. I would like it to be substantive so a reader (me) can really see what you did, what you learned, and how you went a little beyond the minimum!

### Tips

To work with SLAM, the `turtlebot3_slam` package is necessary. To prepare the robot for running SLAM, start up the Turtlebot3 with the `turtlebot3_robot.launch` file. There are a variety of methods that can be used with SLAM, and for this guide we will use `gmapping` to create our map.

#### Tuning SLAM Parameters

To get the most out of `turtlebot3_slam`, some tuning of parameters in the launch file is required. To tune the parameters, open the `turtlebot3_gmapping.launch file` and modify it to look like the following:

<%= include_image "/content/topics/images/slam_launch_file.png" %>

The file above has taken the default configuration, but modified the `map_update_interval`, `linearUpdate`, and `angularUpdate` parameters. Although this may not be the optimal configuration, these parameter updates do significantly increase the map quality from the default launch file. Further expermimentation and tuning of the parameters can be done if results are not satisfactory, and a tuning guide can be found here: http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam

#### Running SLAM

Now that the launch file parameters have been tuned and the Turtlebot3 is running, a map can be created with SLAM. In order to start running SLAM, the following command should be executed on the remote PC connected to roscore:

<%= code_begin %>
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
<%= code_end %>

This should bring up RViz with a view of the map being created. In order to actually create the map, teleop is necessary to drive the robot. In a separate terminal window, run:

<%= code_begin %>
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
<%= code_end %>

Use the keyboard to drive the Turtlebot3 around the space. Once the area has been sufficiently mapped, the map can be saved for use in autonomously navigating.

#### Saving The Map

Once the map has been created using SLAM, it can be saved using the `map_server` package. To save a map, use the command `$ rosrun map_server map_saver -f ~/map`. The `~/map` will save the map with the name `map` to the root directory, but this can be changed by modifying the path on the end of the command. Once this has been run, the map is saved and it can be used for autonomous navigation.

#### Autonomously Navigating

After the map has been successfully saved, the SLAM node which is running can be stopped and navigation can begin. The Turtlebot3 navigation stack contains many powerful navigation algorithms which can be used without any configuration, and we will use one here. To start the navigation node, run the command (with teh correct map file spec)

<%= code_begin %>
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
$ rosrun map_server map_server $HOME/map.yaml
<%= code_end %> 

This file will once again launch RViz, displaying the map which was just created. Before we can navigate, the robot needs to be localized on the map, and this can be done in RViz. Using the `2D Pose Estimate` button, give the robot an estimate of where it is located on the map. Once the robot is in the correct position, a `2D Nav Goal` can be sent through RViz in the same way. With a 2D Nav Goal set, the robot should begin to navigate to the desired location.

#### Summary

Using the same track again, this time there will be obstacles, both stationary and moving and the robot traverses that confidently as well, without colliding with any obstacles. This assignment will combine the previous track following algorithm with AMCL and /scan to also ensure that the robot will not collide with an obstacle. It will drive around it.

##### Equipment

* Track is made with red blocks
* TB3 with Lidar

##### Skills you will learn:

* Using move_base to detect and avoid obstacles
* Debugging a multi-node ROS program

##### Content knowledge you will gain:

* How ROS processes Lidar or visual data
* How ROS reports the presence of obstacles and does short range replanning

##### Examples of this in the teal world
* supply a link or paper or video)

##### What to hand in
* Zip up your source code
* A video (informal) of the robot executing the code

#### Tasks

##### Think through the design
* 

