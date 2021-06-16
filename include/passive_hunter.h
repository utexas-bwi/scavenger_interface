#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "hunter.h"

class PassiveHunter : public Hunter {

private:
	darknet_ros_msgs::BoundingBoxes _bb_msg;
	mutable std::vector<std::string> _image_classes;
	mutable std::vector<std::string> _video_classes;
	ros::Subscriber _image_detection_sub;
	ros::NodeHandle _nh;
	sensor_msgs::Image _rgb_image_msg;
	ros::Subscriber _sensor_image_sub;
	mutable scavenger_hunt_msgs::Hunt _target_hunt;
public:
	PassiveHunter(ros::NodeHandle nh, std::string yolo_bb_topic, std::string rgb_image_topic);	
	void prepare_hunt(scavenger_hunt_msgs::Hunt hunt) const;
	sensor_msgs::Image perform_image_task(scavenger_hunt_msgs::Task) const;
	std::string perform_video_task(scavenger_hunt_msgs::Task) const;
	void perform_find_task(){}
	void perform_location_task(){}
	void perform_travel_task(){}

	/*
	 * Non abstract, Hunter-specific functions
	 */
	void load_classes(scavenger_hunt_msgs::Task task) const;
	void rgb_image_cb(sensor_msgs::Image msg);
	void write_image(sensor_msgs::Image image, std::string path) const;
	void yolo_bb_cb(darknet_ros_msgs::BoundingBoxes msg);
};  
