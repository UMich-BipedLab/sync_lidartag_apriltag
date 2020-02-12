
#include <iostream>
#include <queue>
#include <map>
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
#include <message_filters/sync_policies/approximate_time.h>

#include "lidartag_msgs/LiDARTagDetection.h"
#include "lidartag_msgs/LiDARTagDetectionArray.h"
#include "apriltag_msgs/AprilTagDetection.h"
#include "apriltag_msgs/AprilTagDetectionArray.h"
#include "alignment_msgs/TagDetectionAlignment.h"
#include "alignment_msgs/TagDetectionAlignmentArray.h"

class DetectionAlign{
public:
	DetectionAlign():
	nh_("~"),
	image_seq(0),
	apriltag_seq(0),
	lidartag_seq(0),
	image_received(0),
	apriltag_received(0),
	lidartag_received(0){
		//intilaization
		DetectionAlign::GetParam();
		
    	image_sub = nh_.subscribe(image_topic, 10, &DetectionAlign::imageCallBack, this);
    	apriltag_detection_sub = nh_.subscribe(apriltag_detection_topic, 10, &DetectionAlign::apriltagCallBack, this);
    	lidartag_detection_sub = nh_.subscribe(lidartag_detection_topic, 10, &DetectionAlign::lidartagCallBack, this);	
    	ROS_INFO("Waiting for input data");
		DetectionAlign::_waitForMsg();
	}

	~DetectionAlign(){}; 

	void Run(){
		Init();
		std::cout << "initialized" << std::endl;
		int iter = 0;
		while(ros::ok()){
			boost::mutex::scoped_lock(image_queue_lock);
			boost::mutex::scoped_lock(apriltag_detection_queue_lock);
			boost::mutex::scoped_lock(lidartag_detection_queue_lock);
			boost::mutex::scoped_lock(global_seq_lock);

			if(!image_queue.empty() && !apriltag_detection_queue.empty() && !lidartag_detection_queue.empty()){
				sensor_msgs::Image::ConstPtr img_ptr = image_queue.front();
				apriltag_msgs::AprilTagDetectionArray::ConstPtr apriltag_ptr = apriltag_detection_queue.front();
				lidartag_msgs::LiDARTagDetectionArray::ConstPtr lidartag_ptr = lidartag_detection_queue.front();


				int idxToPub = global_seq.front();
				std::cout << "======= outside ======" << std::endl;
				std::cout << "global_seq :" << idxToPub << std::endl;
				std::cout << "image id : " << img_ptr->header.seq <<std::endl;
				std::cout << "apriltag id : " << apriltag_ptr->frame_index << " Size: " << apriltag_ptr->detections.size() << std::endl;
				std::cout << "lidartag id : " << lidartag_ptr->frame_index << " Size: " << lidartag_ptr->detections.size() << std::endl;

				if (img_ptr->header.seq == idxToPub && 
					apriltag_ptr->frame_index == idxToPub && 
					lidartag_ptr->frame_index == idxToPub){

					// align_image_pub.publish(*img_ptr);
					// align_apriltag_detection_pub.publish(*apriltag_ptr);
					// align_lidartag_detection_pub.publish(*lidartag_ptr);
					publishAlignedMsg(img_ptr, apriltag_ptr, lidartag_ptr);

					image_queue.pop();
					apriltag_detection_queue.pop();
					lidartag_detection_queue.pop();
					global_seq.pop();
				}

				if (img_ptr->header.seq < idxToPub) image_queue.pop();
				if (apriltag_ptr->frame_index < idxToPub) apriltag_detection_queue.pop();
				if (lidartag_ptr->frame_index < idxToPub) lidartag_detection_queue.pop();
				
				if (idxToPub < img_ptr->header.seq || idxToPub < apriltag_ptr->frame_index || idxToPub < lidartag_ptr->frame_index)
					global_seq.pop();

				if (global_seq.empty()){
					global_seq.push(idxToPub + 1);
				}
			}

			ros::spinOnce();
		}
	}

private:
	void GetParam(){
  		nh_.param("image_topic", image_topic, image_topic);
  		nh_.param("apriltag_detection_topic", apriltag_detection_topic, apriltag_detection_topic);
  		nh_.param("lidartag_detection_topic", lidartag_detection_topic, lidartag_detection_topic);

  		nh_.param("align_image_topic", align_image_topic, align_image_topic);
  		nh_.param("align_apriltag_detection_topic", align_apriltag_detection_topic, align_apriltag_detection_topic);
  		nh_.param("align_lidartag_detection_topic", align_lidartag_detection_topic, align_lidartag_detection_topic);
  		nh_.param("alignment_msgs_topic",alignment_msgs_topic, alignment_msgs_topic);
	}


	void Init(){
		align_image_pub = nh_.advertise<sensor_msgs::Image>(align_image_topic,10);
    	align_apriltag_detection_pub = nh_.advertise<apriltag_msgs::AprilTagDetectionArray>(align_apriltag_detection_topic,10);
    	align_lidartag_detection_pub = nh_.advertise<lidartag_msgs::LiDARTagDetectionArray>(align_lidartag_detection_topic, 10);
    	alignment_msgs_pub = nh_.advertise<alignment_msgs::TagDetectionAlignmentArray>(alignment_msgs_topic,10);
	}

	void imageCallBack(const sensor_msgs::Image::ConstPtr& image){
		boost::mutex::scoped_lock(image_queue_lock);
		boost::mutex::scoped_lock(global_seq_lock);
		image_queue.push(image);
		image_received = 1;

		if (!global_seq.empty() && image->header.seq > image_seq+1) //indicating there is a missing message
		{
			if (image->header.seq > global_seq.back()){
				global_seq.push(image->header.seq);
			}
		}	
		image_seq = image->header.seq;
		msg_header = image->header;
        ROS_INFO_STREAM("Image Queue size: " << image_queue.size());
	}


	void apriltagCallBack(const apriltag_msgs::AprilTagDetectionArray::ConstPtr& apriltag_detection){
		boost::mutex::scoped_lock(apriltag_detection_queue_lock);
		boost::mutex::scoped_lock(global_seq_lock);
		apriltag_detection_queue.push(apriltag_detection);
		apriltag_received =1;

		if (!global_seq.empty() && apriltag_detection->frame_index > apriltag_seq+1 && apriltag_detection->frame_index > global_seq.back()){
			global_seq.push(apriltag_detection->frame_index);
		}
		apriltag_seq = apriltag_detection->frame_index;

        ROS_INFO_STREAM("Apriltag Queue size: " << apriltag_detection_queue.size());
	}


	void lidartagCallBack(const lidartag_msgs::LiDARTagDetectionArray::ConstPtr& lidartag_detection){
		boost::mutex::scoped_lock(lidartag_detection_queue_lock);
		boost::mutex::scoped_lock(global_seq_lock);

		lidartag_detection_queue.push(lidartag_detection);
		lidartag_received = 1;
		ROS_INFO_STREAM("LiDARTagDetection received");

		if (!global_seq.empty() && lidartag_detection->frame_index > lidartag_seq+1 && lidartag_detection->frame_index > global_seq.back()){
			global_seq.push(lidartag_detection->frame_index);
		}

		lidartag_seq = lidartag_detection->frame_index;
        ROS_INFO_STREAM("Lidar Queue size: " << lidartag_detection_queue.size());
	}
 
    void _waitForMsg(){
        while (ros::ok()){
            if (image_received && apriltag_received && lidartag_received) {
                ROS_INFO("Got all messages");
                boost::mutex::scoped_lock(global_seq_lock);
                global_seq.push(0);
                return;
            }
            ros::spinOnce();
        }
    }

    void publishAlignedMsg(sensor_msgs::Image::ConstPtr img_ptr,
						   apriltag_msgs::AprilTagDetectionArray::ConstPtr apriltag_ptr,
						   lidartag_msgs::LiDARTagDetectionArray::ConstPtr lidartag_ptr){
    	
    	alignment_msgs::TagDetectionAlignmentArray aligned_detections;
    	aligned_detections.header = msg_header;
    	aligned_detections.frame_index = apriltag_ptr->frame_index;
    	
    	std::map<int, lidartag_msgs::LiDARTagDetection> lidarmap;
    	std::map<int, apriltag_msgs::AprilTagDetection> aprilmap;

    	for(const auto &tag : lidartag_ptr->detections){
    		if(lidarmap.count(tag.id)) lidarmap.erase(tag.id);
    		else lidarmap[tag.id] = tag;
    	}
    	

    	for(const auto &tag : apriltag_ptr->detections){
    		if(aprilmap.count(tag.id)) aprilmap.erase(tag.id);
    		else aprilmap[tag.id] = tag;
    	}

    	int count =0;
    	for(const auto &iter : lidarmap){
    		int id = iter.first;
    		if (aprilmap.count(id)){
    	    	alignment_msgs::TagDetectionAlignment detection;
	    		detection.header = msg_header;
	    		detection.frame_index = apriltag_ptr->frame_index;
	    		detection.image = *img_ptr;
    			detection.apriltag_detection = aprilmap[id];
    			detection.lidartag_detection = iter.second;
	    		aligned_detections.detections.push_back(detection);
    		}

    	}
    	std::cout << "tag alignment message size: " << aligned_detections.detections.size() <<std::endl;
    	alignment_msgs_pub.publish(aligned_detections);
    }


private:
	ros::NodeHandle nh_;

	ros::Subscriber image_sub;
	ros::Subscriber apriltag_detection_sub;
	ros::Subscriber lidartag_detection_sub;

	ros::Publisher align_image_pub;
	ros::Publisher align_apriltag_detection_pub;
	ros::Publisher align_lidartag_detection_pub;
	ros::Publisher alignment_msgs_pub;
	//parameters
	std::string image_topic;
	std::string apriltag_detection_topic;
	std::string lidartag_detection_topic;

	std::string align_image_topic;
	std::string align_apriltag_detection_topic;
	std::string align_lidartag_detection_topic;
	std::string alignment_msgs_topic;

	//queue
	std::queue<apriltag_msgs::AprilTagDetectionArray::ConstPtr> apriltag_detection_queue;
	std::queue<lidartag_msgs::LiDARTagDetectionArray::ConstPtr> lidartag_detection_queue;
	std::queue<sensor_msgs::Image::ConstPtr> image_queue;

	//indexing
	std::queue<int> global_seq;
	int image_seq;
	int apriltag_seq;
	int lidartag_seq;

	//lock
	boost::mutex global_seq_lock;
	boost::mutex image_queue_lock;
	boost::mutex apriltag_detection_queue_lock;
	boost::mutex lidartag_detection_queue_lock;

	//flag 
	bool image_received;
	bool apriltag_received;
	bool lidartag_received;

	std_msgs::Header msg_header;
};

int main(int argc, char **argv){
	ROS_INFO("detection_align_node init");
    ros::init(argc, argv, "detection_align_node");

    DetectionAlign detection_align_node;
    detection_align_node.Run();

    ros::shutdown();
    return 0;
}