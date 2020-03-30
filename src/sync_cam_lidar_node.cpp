 /* 
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu) and Chenxi Feng (chenxif@umich.edu)
 * WEBSITE: https://www.brucerobot.com/
*/

#include <iostream>
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

class CamLidarSync{
public:
	CamLidarSync():
	private_nh_("~"){
		//intilaization
		CamLidarSync::GetParam();
		
    	image_sub.subscribe(nh_, image_topic, 10);
    	pointcloud_sub.subscribe(nh_, pointcloud_topic, 10);
		sync_policy = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_sub, pointcloud_sub);
    	sync_policy->registerCallback(boost::bind(&CamLidarSync::callback, this, _1, _2));
	}

	~CamLidarSync(){}; 

	void Run(){
		Init();
		ros::spin();
	}

private:
	void GetParam(){
		private_nh_.param("pointcloud_topic", pointcloud_topic, pointcloud_topic);
  		private_nh_.param("image_topic", image_topic, image_topic);
  		private_nh_.param("sync_pointcloud_topic", sync_pointcloud_topic, sync_pointcloud_topic);
  		private_nh_.param("sync_image_topic", sync_image_topic, sync_image_topic);
	}


	void Init(){
		image_pub = nh_.advertise<sensor_msgs::Image>(sync_image_topic,10);
    	pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(sync_pointcloud_topic,10);
	}

	void callback(const sensor_msgs::Image::ConstPtr& image, 
				  const sensor_msgs::PointCloud2::ConstPtr& point_cloud){
		image_pub.publish(*image);
		pointcloud_pub.publish(*point_cloud);
	}


private:
	ros::NodeHandle nh_, private_nh_;
	ros::Publisher image_pub;
	ros::Publisher pointcloud_pub;
	message_filters::Subscriber<sensor_msgs::Image>  image_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
														    sensor_msgs::PointCloud2> SyncPolicy;
	message_filters::Synchronizer<SyncPolicy> *sync_policy;
	//parameters
	std::string pointcloud_topic;
	std::string image_topic;
	std::string sync_pointcloud_topic;
	std::string sync_image_topic;
};

int main(int argc, char **argv){
	ROS_INFO("sync_cam_lidar_node init");
    ros::init(argc, argv, "sync_cam_lidar_node");

    CamLidarSync sync_cam_lidar_node;
    sync_cam_lidar_node.Run();

    ros::shutdown();
    return 0;
}
