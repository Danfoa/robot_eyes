# Robot Eyes

(Work in progress...see [Wiki](https://github.com/Danfoa/robot_eyes/wiki/) for further documentation))

This package intents to create a modular system for RGB-D and/or RGB image processing, integrating state of the art research and vision frameworks like PCL.

### Project Charactersistics 
- [ ] Efficienty intead of univerasitlity: We want to create a system that can process effectively a certain number of 'known' objects, i.e. we want to deploy robotic industrial applications meaning that the scenareos of the robot will not be totally unknown.

- [ ] Modulatiry.

- [ ] Real time operation.
 
 
Initially we want to achieve a modular design for: 
 * Point cloud pre processing
 * Dominant plane detection (not focusing only to tables)
 * Point cloud grouping (oct-threes, n-threes, K-nearest neightboors, approximation to K-nearest...etc)
 * Object detection
 * Pose estimation (from known CAD files) 
 * Scene and shape completition (with geometric primitives of semantic information)


## Nodes
* Segmentation:
    * [Plane Segmenter](https://github.com/Danfoa/robot_eyes/wiki/plane_segmenter): This Node uses RANSAC algorithm to find planar component in the cloud.
* Visualization:
    * [Cloud Colorer](https://github.com/Danfoa/robot_eyes/wiki/cloud_colorer): This Node color clusters of a cloud for visualization purposes.
