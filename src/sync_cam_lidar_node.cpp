
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
    	image_sub.subscribe(nh_, "/camera/color/image_raw", 10);
    	pointcloud_sub.subscribe(nh_, "/velodyne_points", 10);

		sync_policy = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_sub, pointcloud_sub);
    	// sync_policy_.connectInput(image_sub,pointcloud_sub);
    	sync_policy->registerCallback(boost::bind(&CamLidarSync::callback, this, _1, _2));	
	}

	~CamLidarSync(){}; 

	void Run(){
		Init();
		ros::spin();
	}

private:
	void Init(){
		image_pub = nh_.advertise<sensor_msgs::Image>("/camera/color/image_sync",10);
    	pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_sync",10);
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
};

int main(int argc, char **argv){
	ROS_INFO("sync_cam_lidar_node init");
    ros::init(argc, argv, "sync_cam_lidar_node");

    CamLidarSync sync_cam_lidar_node;
    sync_cam_lidar_node.Run();

    ros::shutdown();
    return 0;
}