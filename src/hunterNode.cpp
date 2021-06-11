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
#include "scavenger_ros_interface.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "scavenger_hunter");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// Fetch all required information on the parameter server
	std::string getHuntSrv, sendProofSrv, getProofSrv, huntName, imageDetectionTopic, rgbImageTopic;
		
	nh.getParam("get_hunt_srv", getHuntSrv);
	nh.getParam("send_proof_srv", sendProofSrv);
	nh.getParam("get_proof_srv", getProofSrv);
	nh.getParam("hunt_name", huntName);
	nh.getParam("image_detection_topic", imageDetectionTopic);
	nh.getParam("image_topic", rgbImageTopic);

	ScavengerRosInterface* sri = new ScavengerRosInterface(nh, getHuntSrv, sendProofSrv, getProofSrv);

	// fetch the hunt and exit if it's not found
	scavenger_hunt_msgs::Hunt hunt = sri->getHunt(huntName);
	if( hunt == scavenger_hunt_msgs::Hunt()){
		ROS_ERROR("Scavenger hunt with name: %s  was not retreived from the scavenger server. Quitting.", huntName.c_str());
		return -1;
	}

	PassiveHunter* ph = new PassiveHunter(nh, imageDetectionTopic, rgbImageTopic);
	
	ph->prepareHunt(hunt);
	
	// iterate over tasks and sequentially submit to Hunter
	for( scavenger_hunt_msgs::Task t : hunt.tasks){
		bool success = true;
		if(t.proof_format == std::to_string(scavenger_hunt_msgs::Proof::TYPE_IMAGE)){
			ros::Time start = ros::Time::now();
			sensor_msgs::Image imageProof = ph->performImageTask(t);
			ROS_INFO("Proof has been found. Attempting to submit.");
			success = sri->uploadProof(t,imageProof,start);
		}
		else if(t.proof_format == std::to_string(scavenger_hunt_msgs::Proof::TYPE_VIDEO)){
			ros::Time start = ros::Time::now();
			std::string proofPath = ph->performVideoTask(t);
			ROS_INFO("Proof has been found. Attempting to submit.");
			success = sri->uploadProof(t,proofPath,start);
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

	return 0;
}
