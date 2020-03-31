# sync_lidartag_apriltag

## Overview
This is a synchronization package for [LiDARTags](https://github.com/UMich-BipedLab/LiDARTag) and [AprilTags](https://github.com/UMich-BipedLab/AprilTag_ROS). As the LiDAR and camera data streams arrive, they are synchronized (synchronization node), examined for fiducial markers (LiDARTags and AprilTags), and checked for false positives (tag-pairing node). The relevant information is saved as ROS bagfiles from _alignment_msgs_ for post-processing in another [package](https://github.com/UMich-BipedLab/automatic_lidar_camera_calibration). For LiDAR-Camera extrinsic calibration, we use target vertices and image corners as our features. 

**Author: Bruce JK Huang
Maintainer: Bruce JK Huang, bjhuang@umich.edu  
Affiliation: The Biped Lab, the University of Michigan**

This package has been tested under MATLAB2019a and Ubuntu 16.04.


## Required packages
Please download _alignment_msgs_ from [here](https://github.com/UMich-BipedLab/alignment_msgs). 
Please download _LiDARTag_ from [here](https://github.com/UMich-BipedLab/LiDARTag).  
Please download _LiDARTag_msgs_ from [here](https://github.com/UMich-BipedLab/LiDARTag_msgs).  
Please download _AprilTag_ from [here](https://github.com/UMich-BipedLab/AprilTag_ROS).  
Please download _AprilTag_msgs_ from [here](https://github.com/UMich-BipedLab/apriltag_msgs).  


## Input/Output
Input: LiDAR piont clouds and camera images publish via ROS.
Output: _alignment_msgs_ containing relevant information such as AprilTag corners and LiDARTag target points.

<!--
In the LiDARTag and AprilTag packages, detection messages will be published even if no tag is detected (i.e., published an empty message). Thus, there is no missing message at this step. Additionally, each detection message is labeled with a frame index from the corresponding synced image. The alignment step (tag pairing) handles false positive in the process of detection of fiducial marke. We use the frame index to track, and group these detection messages. We do the id association and find the paired targets. 
If there is no paired tags, this message will be abandoned. To deal with the time delay between different types of messages(image, Lidartag detection, Apriltag detection), we added three queues for each of them. In addition, we added a safe check in the tag alignment in case there is an unexpected missing message. The way we do safe check is using queues to store the messages. The front of global queue will be the candidate index to publish. If there is one message missing, we need to consider two cases: 1, If the global queue is working properly, it will detect the a jump in one of the message and don't have that missing index in the queue. Then we will pop out the messages with frame index smaller than the candidate index. 2. The global queue doesn't detect the missing index and still have that missing index in queue. Then, at one time the candidate index will be smaller than the frame index of one of  three messages. In this case, we pop out the candidate frame index from the global queue and return to case 1. 
-->


## Usage
This package contains three launch files.
The _sync_cam_lidar_ launch file will run the data synchronization node, launch the LiDARTag and AprilTag nodes. 
The _alignment_node_only_ launch file will run the tag pairing node, which detect false positives of the LiDARTags and AprilTags.
The _sync_node_only_ launch file will only run the data synchronization node, which synchronizes LiDAR point clouds and camera images. 


## Usage for automatic calibration
To compete the fron-end of the pipeline of the [automatic calibration](https://github.com/UMich-BipedLab/automatic_lidar_camera_calibration), the
_sync_cam_lidar_ launch file should be ran first, and then run the
_alignment_node_only_ launch file to run the tag pairing node. 
