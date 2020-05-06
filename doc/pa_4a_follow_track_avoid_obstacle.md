### Summary

This programming assignment builds on the *follow_track* assignment. The assignment is the same, except that now there will be an obstacle somewhere on the track. The obstacle will be rectangular and 50% wider than the track. Your robot will use the Lidar (or other sensor) to detect the obstacle, and drive around it before continuing.

### Skills you will learn:

* Thinking about simple navigation algorithms and how they may work or go wrong. Thinking about their edge conditions.
* How to use noisy lidar data to detect an obstacle in the path of the robot. This will obviously build on several of the previous assignments.

### Content knowledge you will gain
* 

* 

### Examples of this in the real world
* supply a link or paper or video)

### What to hand in
* Zip up your source code
* A video (informal) of the robot executing the code

### Tasks
* Test your line follower from <%= link_to_topic :follow_track %> to make sure it still works.
* Create a subscriber node to process the lidar data
* Update your line follower node to change behavior if there is an obstacle

