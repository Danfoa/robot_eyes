# Robot Eyes
(Work in progress...)

ROS package for real time object detection/segmention and pose estimation to achieve grasping and traking operations.

## Desired Outcome
This package intents to create a modular system for RGB-D and/or RGB image processing, initially a similar result as in [pr2_object_manipulation repository](http://wiki.ros.org/pr2_object_manipulation?distro=groovy) but with an small diffent approach.

### Project Charactersistics 
- [ ] Efficienty intead of univerasitlity: We want to create a system that can process effectively a certain number of 'known' objects, i.e. we want to deploy robotic industrial applications meaning that the scenareos of the robot will not be totally unknown.

- [ ] Modulatiry: Meaning that if you have a better idea in how to do one of the operations you will be able to integrate it into the processing pipeline easily. This is desired in order to keep up with state of the art real time algorithms and manage the flexibility to operate for different robot manipulators, and because to the Authors knowledge there is not such a project like this...if you know of one like this please let us known :P ... we off course want to work on top of PCL already developed algorithms.

- [ ] Real time operation: Well you already understand this one ;)
 
 
We want to achieve a modular design for: 
 * Point cloud pre processing
 * Dominant plane detection (not focusing only to tables)
 * Point cloud grouping (oct-threes, n-threes, K-nearest neightboors, approximation to K-nearest...etc)
 * Object detection
 * Pose estimation (from known CAD files) 
 * Scene and shape completition (with geometric primitives of semantic information)


## TO-DO
- [ ] Find cooler name
- [ ] Create first tabletop object detection/manipulation application
- [ ] Standarize further development and contributions
The `Projects` tab should standarize more the goals and further development.
## Origin
This is a project of the National University of Colombia.
