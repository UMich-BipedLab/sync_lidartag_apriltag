# sync_cam_lidar

## Overview
This is a synchronization package for LiDARTags and AprilTag. 

**Author: Bruce JK Huang
Maintainer: Bruce JK Huang, bjhuang@umich.edu  
Affiliation: The Biped Lab, the University of Michigan**

This package has been tested under MATLAB2019a and Ubuntu 16.04.


## Required packages
Please download _LiDARTag_ from [here](https://github.com/UMich-BipedLab/LiDARTag).  
Please download _LiDARTag_msgs_ from [here](https://github.com/UMich-BipedLab/LiDARTag_msgs).  
Please download _AprilTag_ from [here](https://github.com/UMich-BipedLab/AprilTag_ROS).  
Please download _AprilTag_msgs_ from [here](https://github.com/UMich-BipedLab/apriltag_msgs).  

## Usage
This package contains three launch files.
The _sync_cam_lidar_ launch file will run the data synchronization node, launch the LiDARTag and AprilTag nodes. 
The _alignment_node_only_ launch file will run the tag pairing node, which detect false positives of the LiDARTags and AprilTags.
The _sync_node_only_ launch file will only run the data synchronization node, which synchronizes LiDAR point clouds and camera images. 


## Usage for automatic calibration
To compete the fron-end of the pipeline of the automatic calibration, the
_sync_cam_lidar_ launch file should be ran first, and then run the
_alignment_node_only_ launch file to run the tag pairing node. 
