/* 
 * An example of a Scavenger Hunter that runs passively, without strictly controlling 
 * the robot's actions to perform or complete tasks. Thus, it is the easiest to get 
 * running on a new robot platform, but may not be high scoring, depending on the 
 * current actions of the robot.
 *
 * At this time YOLO is required for image / class detection, and only IMAGE tasks 
 * are supported.
 *
 * Author Maxwell Svetlik, 2021
 *
 */

#include <algorithm>
#include <darknet_ros_msgs/BoundingBox.h>
#include <scavenger_hunt_msgs/Proof.h>
#include <string>

#include "passive_hunter.h"
#include "scavenger_constants.h"

PassiveHunter::PassiveHunter(ros::NodeHandle nh, std::string yoloBBTopic, std::string rgbImageTopic) : Hunter(){
	_nh = nh;
	_bbMsg = darknet_ros_msgs::BoundingBoxes();
	_rgbImageMsg = sensor_msgs::Image();
	_imageDetectionSub = _nh.subscribe(yoloBBTopic, 20, &PassiveHunter::yoloBBCb, this);
	_sensorImageSub = _nh.subscribe(rgbImageTopic, 20, &PassiveHunter::rgbImageCb, this);
}

void PassiveHunter::prepareHunt(scavenger_hunt_msgs::Hunt hunt) const {

	_targetHunt = hunt;
	_imageClasses.clear();
	_videoClasses.clear();
	for(scavenger_hunt_msgs::Task t : hunt.tasks){
		loadClasses(t);
	}

}

sensor_msgs::Image PassiveHunter::performImageTask(scavenger_hunt_msgs::Task task) const { 
	
	_imageClasses.clear();
	loadClasses(task);
	ros::Rate r(10);
	ROS_INFO("Performing an Image task");
	while(ros::ok()){
		for(darknet_ros_msgs::BoundingBox bb : _bbMsg.bounding_boxes){
			if (std::find(_imageClasses.begin(), _imageClasses.end(), bb.Class) != _imageClasses.end()){
				return _rgbImageMsg;
			}
		}
		r.sleep();
		ros::spinOnce();
	}

	return sensor_msgs::Image();
}

std::string PassiveHunter::performVideoTask(scavenger_hunt_msgs::Task task) const { 
	return "";
}


/*
 * Non abstract, Hunter-specific functions
 */


// Since this Hunter is passive, collect all class labels that we might be looking for
// across all tasks to achieve in parallel
void PassiveHunter::loadClasses(scavenger_hunt_msgs::Task task) const {
	if(task.proof_format == IMAGE){
		for(scavenger_hunt_msgs::Parameter p : task.parameters)
			_imageClasses.push_back(p.value);
	}
	else if(task.proof_format == VIDEO){
		for(scavenger_hunt_msgs::Parameter p : task.parameters)
			_videoClasses.push_back(p.value);
	}
}


void PassiveHunter::rgbImageCb(sensor_msgs::Image msg){
	_rgbImageMsg = msg;
}

void PassiveHunter::yoloBBCb(darknet_ros_msgs::BoundingBoxes msg){
	_bbMsg = msg;
}
