/*
 * A platform to kick off and submit tasks to generic scavenger Hunters.
 * This is the primary node that is brought up and connects the 
 * ScavengerInterface library with a Hunter of your choice. 
 *
 * By default the Hunter that is instantiated is the PassiveHunter, a Hunter that 
 * monitors for solutions to tasks without actually taking specific actions to 
 * complete the task.
 *
 * Author Maxwell Svetlik, 2021
 *
 */

#include <ros/ros.h>
#include <scavenger_hunt_msgs/Proof.h>

#include "passive_hunter.h"
#include "scavenger_constants.h"
#include "scavenger_ros_interface.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "scavenger_hunter");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(10);

	// Fetch all required information on the parameter server
	std::string get_hunt_srv, send_proof_srv, get_proof_srv, hunt_name, image_detection_topic, rgb_image_topic;
		
	nh.getParam("get_hunt_srv", get_hunt_srv);
	nh.getParam("send_proof_srv", send_proof_srv);
	nh.getParam("get_proof_srv", get_proof_srv);
	nh.getParam("hunt_name", hunt_name);
	nh.getParam("image_detection_topic", image_detection_topic);
	nh.getParam("image_topic", rgb_image_topic);

	ScavengerRosInterface* sri = new ScavengerRosInterface(nh, get_hunt_srv, send_proof_srv, get_proof_srv);

	// fetch the hunt and exit if it's not found
	scavenger_hunt_msgs::Hunt hunt = sri->get_hunt(hunt_name);
	if( hunt == scavenger_hunt_msgs::Hunt()){
		ROS_ERROR("Scavenger hunt with name: %s  was not retreived from the scavenger server. Quitting.", hunt_name.c_str());
		return -1;
	}

	PassiveHunter* ph = new PassiveHunter(nh, image_detection_topic, rgb_image_topic);
	
	ph->prepare_hunt(hunt);
	
	// iterate over tasks and sequentially submit to Hunter
	for( scavenger_hunt_msgs::Task t : hunt.tasks){
		bool success = true;
		if(t.proof_format == IMAGE){
			ros::Time start = ros::Time::now();
			sensor_msgs::Image image_proof = ph->perform_image_task(t);
			ROS_INFO("Proof has been found. Attempting to submit.");
			success = sri->upload_proof(t,image_proof,start);
		}
		else if(t.proof_format == VIDEO){
			ros::Time start = ros::Time::now();
			std::string proof_path = ph->perform_video_task(t);
			ROS_INFO("Proof has been found. Attempting to submit.");
			success = sri->upload_proof(t,proof_path,start);
		}
		else{
			ROS_WARN("Task proof format not known or unsupported. Skipping...");
			continue;
		}

		if( !success )
			ROS_ERROR("Submitting proof to Scavenger server failed.");
		else
			ROS_INFO("Proof submitted successfully.");

	}

	ROS_INFO("Completed all tasks associated with Hunt. Hunt complete!");
	return 0;
}
