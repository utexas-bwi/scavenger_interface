/*
 * A ROS library that wraps around the Scavenger Hunt API to simplify common interactions 
 * such as fetching tasks and uploading proofs.
 * 
 * Author Maxwell Svetlik, 2021
 */

#include <scavenger_hunt_msgs/GetHunt.h>
#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/Proof.h>
#include <scavenger_hunt_msgs/SendProof.h>

#include "scavenger_ros_interface.h"


ScavengerRosInterface::ScavengerRosInterface(ros::NodeHandle nh, std::string getHuntSrv, std::string sendProofSrv, std::string getProofSrv){
	_nh = nh;
	_isOk = true;

	// wait for services to be advertised
	if( 	!ros::service::waitForService(getHuntSrv, ros::Duration(5.0)) ||
		!ros::service::waitForService(sendProofSrv, ros::Duration(5.0)) ||
		!ros::service::waitForService(getProofSrv, ros::Duration(5.0)) ){

		ROS_WARN("One or more Scavenger services are not availble. Scavenger Hunt API will not work as expected.");
		_isOk = false;
	}
	_getHuntClient = _nh.serviceClient<scavenger_hunt_msgs::GetHunt>(getHuntSrv);
	_sendProofClient = _nh.serviceClient<scavenger_hunt_msgs::SendProof>(sendProofSrv);
	_getStatusClient = nh.serviceClient<scavenger_hunt_msgs::GetProofStatus>(getProofSrv);

}

scavenger_hunt_msgs::Hunt ScavengerRosInterface::getHunt(std::string huntName){

	scavenger_hunt_msgs::GetHunt getHunt;
	getHunt.request.hunt_name = huntName;
	bool success = _getHuntClient.call(getHunt);
	if (!success){
		ROS_ERROR("Call to GetHunt service failed.");
		return scavenger_hunt_msgs::Hunt();
	}
	return getHunt.response.hunt;	
}

std::vector<scavenger_hunt_msgs::Task> ScavengerRosInterface::getTasks(std::string huntName){
	scavenger_hunt_msgs::Hunt hunt = getHunt(huntName);
	return hunt.tasks;
}

unsigned char ScavengerRosInterface::getHuntStatus(unsigned long proofId){
	scavenger_hunt_msgs::GetProofStatus getStatus;
	getStatus.request.id = proofId;

	_getStatusClient.call(getStatus);
	// 0 -> Proof is not yet validated
	// 1 -> Proof was marked correct
	// 2 -> Proof was marked incorrect
	return getStatus.response.status;
}

bool ScavengerRosInterface::isHuntValidated(unsigned long proofId){
	return getHuntStatus(proofId) != '0';
}

bool ScavengerRosInterface::isHuntCorrect(unsigned long proofId){
	return getHuntStatus(proofId) == '1';
}

bool ScavengerRosInterface::isHuntIncorrect(unsigned long proofId){
	return getHuntStatus(proofId) == '2';
}

bool ScavengerRosInterface::isOk(){
	return _isOk;
}

/*
 * Send an image proof to the server given the image and the task its associated with
 * Returns true on success, false otherwise.
 */

bool ScavengerRosInterface::uploadProof(scavenger_hunt_msgs::Task task, sensor_msgs::Image image, ros::Time startTime){
	scavenger_hunt_msgs::Proof proof;
	proof.image = image;
	proof.task_duration = ros::Time::now().toSec() - startTime.toSec();
	scavenger_hunt_msgs::SendProof sendProof;
	sendProof.request.proof = proof;
	sendProof.request.task = task;

	bool success = _sendProofClient.call(sendProof);
	if (!success){
		ROS_ERROR("Call to UploadProof service failed. Proof not uploaded.");
		return false;
	}
	unsigned long proofId = sendProof.response.id;
	if (proofId == -1){
		ROS_ERROR("Upload request to API failed. Proof not uploaded.");
		return false;
	}
	return true;
}

/*
 * Send a video proof to the server given the path the .mp4 file and the task its associated with
 */

bool ScavengerRosInterface::uploadProof(scavenger_hunt_msgs::Task task, std::string filePath, ros::Time startTime){
	if (!fileAccessible(filePath)){
		ROS_ERROR("File path %s is not accessible. Abortting attempt to upload proof.", filePath.c_str());
		return false;
	}
	
	scavenger_hunt_msgs::Proof proof;
	proof.type = scavenger_hunt_msgs::Proof::TYPE_VIDEO;
	proof.file_path = filePath;
	proof.task_duration = ros::Time::now().toSec() - startTime.toSec();
	scavenger_hunt_msgs::SendProof sendProof;
	sendProof.request.proof = proof;
	sendProof.request.task = task;

	bool success = _sendProofClient.call(sendProof);
	if (!success){
		ROS_ERROR("Call to UploadProof service failed. Proof not uploaded.");
		return false;
	}
	unsigned long proofId = sendProof.response.id;
	if (proofId == -1){
		ROS_ERROR("Upload request to API failed. Proof not uploaded.");
		return false;
	}
	return true;
}
