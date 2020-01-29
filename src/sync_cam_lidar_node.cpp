
#include <iostream>
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
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
	message_filters::Subscriber<sensor_msgs::Image> image_sub;
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