#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>

#include "hunter.h"

class PassiveHunter : public Hunter {

private:
	darknet_ros_msgs::BoundingBoxes _bbMsg;
	mutable std::vector<std::string> _imageClasses;
	mutable std::vector<std::string> _videoClasses;
	ros::Subscriber _imageDetectionSub;
	ros::NodeHandle _nh;
	sensor_msgs::Image _rgbImageMsg;
	ros::Subscriber _sensorImageSub;
	mutable scavenger_hunt_msgs::Hunt _targetHunt;
public:
	PassiveHunter(ros::NodeHandle nh, std::string yoloBBTopic, std::string rgbImageTopic);	
	void prepareHunt(scavenger_hunt_msgs::Hunt hunt) const;
	sensor_msgs::Image performImageTask(scavenger_hunt_msgs::Task) const;
	std::string performVideoTask(scavenger_hunt_msgs::Task) const;
	void performFindTask(){}
	void performLocationTask(){}
	void performTravelTask(){}

	/*
	 * Non abstract, Hunter-specific functions
	 */
	void loadClasses(scavenger_hunt_msgs::Task task) const;
	void rgbImageCb(sensor_msgs::Image msg);
	void yoloBBCb(darknet_ros_msgs::BoundingBoxes msg);
};  
