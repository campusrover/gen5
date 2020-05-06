#### Summary

We have an outdoor track which is about 10 meters long. It is marked by a series of orange cones. The goal is to write or extend what you have written so far to allow the robot to follow this path, using simple computer vision from one end to the other, then rotate in place and drive back.

##### Equipment
* CR4 with a depth camera
* Outdoor path marked with cones

##### Skills you will learn:

* Processing information coming from the robot camera
* Using CV algorithms to identify the location and distance of the cones
* Using a <xxx> algorithm to compute the best next point to navigate to in order to remain on the path
* Using the robot outdoors where the path is rough, there is sunlight, and other annoying things

##### Content knowledge you will gain:

* Commputer Vision with OpenCV
* Key functions and technoques for processing images
* What the data coming from a camera and a depth camera looks like and how to interpret it.

##### Examples of this in the real world
* supply a link or paper or video)

##### What to hand in
* Zip up your source code
* A video (informal) of the robot executing the code

#### Tasks

##### Think through the design
* We can assume that the robot starts "on the path"
* The depth camera will give an image of what it sees, including a distance from the camera of each pixel
* From this you need to come up with a 2-dimenstional projection of the detected conest onto the ground plane.
  * Does the orientation of the robot's camera matter?
  * What is the origin and orientation of the 2-d coordinate system? Is it the robot itself?
  * Is it ok if the robot turns in place. Do all the coordinates change?
* Now we use the <xxx> algorithm to detect what the path between the visible cones is
* And compute a direction and velocity for the robot
* This gets recomputed multiple times per second

##### The Key Algorithm

