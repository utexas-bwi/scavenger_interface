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

PassiveHunter::PassiveHunter(ros::NodeHandle nh, std::string yolo_bb_topic, std::string rgb_image_topic) : Hunter(){
	_nh = nh;
	_bb_msg = darknet_ros_msgs::BoundingBoxes();
	_rgb_image_msg = sensor_msgs::Image();
	_image_detection_sub = _nh.subscribe(yolo_bb_topic, 20, &PassiveHunter::yolo_bb_cb, this);
	_sensor_image_sub = _nh.subscribe(rgb_image_topic, 20, &PassiveHunter::rgb_image_cb, this);
}

void PassiveHunter::prepare_hunt(scavenger_hunt_msgs::Hunt hunt) const {

	_target_hunt = hunt;
	_image_classes.clear();
	_video_classes.clear();
	for(scavenger_hunt_msgs::Task t : hunt.tasks){
		load_classes(t);
	}
}

sensor_msgs::Image PassiveHunter::perform_image_task(scavenger_hunt_msgs::Task task) const { 
	
	_image_classes.clear();
	load_classes(task);
	ros::Rate r(10);
	ROS_INFO("Performing an Image task");
	while(ros::ok()){
		for(darknet_ros_msgs::BoundingBox bb : _bb_msg.bounding_boxes){
			if (std::find(_image_classes.begin(), _image_classes.end(), bb.Class) != _image_classes.end()){
				return _rgb_image_msg;
			}
		}
		r.sleep();
		ros::spinOnce();
	}

	return sensor_msgs::Image();
}

/*
 * Record images to scratch directory, then assemble them
 * into a video. Returns path to video.
 *
 */
std::string PassiveHunter::perform_video_task(scavenger_hunt_msgs::Task task) const { 
	float video_duration = 5.0;
	int frame_rate = 10;
	_video_classes.clear();
	load_classes(task);
	ros::Rate r(frame_rate);
	ROS_INFO("Performing a Video task");
	while(ros::ok()){
		for(darknet_ros_msgs::BoundingBox bb : _bb_msg.bounding_boxes){
			/*
			 * Note that *what* triggers the video is dependent on the task. For now 
			 * the video starts once an image containing an object is found. 
			 */
			if (std::find(_video_classes.begin(), _video_classes.end(), bb.Class) != _video_classes.end()){
				std::string scratch_path;
				int counter = 1000;
				_nh.param("scavenger_hunt/scratch_path", scratch_path, std::string("~/"));
				ros::Time video_start = ros::Time::now();
				while( ros::Time::now() - video_start < ros::Duration(video_duration)){
					write_image(_rgb_image_msg, scratch_path + "frame" + std::to_string(counter) + ".jpg");
					counter += 1;
					r.sleep();
					ros::spinOnce();
				}
				
				// Create video file
				std::string cmd = "ffmpeg -framerate " + std::to_string(frame_rate) + " -i " + scratch_path + "frame1%0d3.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p " + scratch_path + "scavenger_output.mp4";
				int success = system(cmd.c_str());
				return scratch_path + "scavenger_output.mp4";
			}
		}
		r.sleep();
		ros::spinOnce();
	}
	return "";
}


/*
 * Non abstract, Hunter-specific functions
 */


// Since this Hunter is passive, collect all class labels that we might be looking for
// across all tasks to achieve in parallel
void PassiveHunter::load_classes(scavenger_hunt_msgs::Task task) const {
	if(task.proof_format == IMAGE){
		for(scavenger_hunt_msgs::Parameter p : task.parameters)
			_image_classes.push_back(p.value);
	}
	else if(task.proof_format == VIDEO){
		for(scavenger_hunt_msgs::Parameter p : task.parameters)
			_video_classes.push_back(p.value);
	}
}

void PassiveHunter::write_image(sensor_msgs::Image image, std::string path) const {
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	cv::Mat image_raw = cv_ptr->image;
	cv::imwrite(path, image_raw);
}

void PassiveHunter::rgb_image_cb(sensor_msgs::Image msg){
	_rgb_image_msg = msg;
}

void PassiveHunter::yolo_bb_cb(darknet_ros_msgs::BoundingBoxes msg){
	_bb_msg = msg;
}
