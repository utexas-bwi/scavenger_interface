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


ScavengerRosInterface::ScavengerRosInterface(ros::NodeHandle nh, std::string get_hunt_srv, std::string send_proof_srv, std::string get_proof_srv){
	_nh = nh;
	_is_ok = true;

	// wait for services to be advertised
	if( 	!ros::service::waitForService(get_hunt_srv, ros::Duration(5.0)) ||
		!ros::service::waitForService(send_proof_srv, ros::Duration(5.0)) ||
		!ros::service::waitForService(get_proof_srv, ros::Duration(5.0)) ){

		ROS_WARN("One or more Scavenger services are not availble. Scavenger Hunt API will not work as expected.");
		_is_ok = false;
	}
	_get_hunt_client   = _nh.serviceClient<scavenger_hunt_msgs::GetHunt>(get_hunt_srv);
	_send_proof_client = _nh.serviceClient<scavenger_hunt_msgs::SendProof>(send_proof_srv);
	_get_status_client = _nh.serviceClient<scavenger_hunt_msgs::GetProofStatus>(get_proof_srv);

}

scavenger_hunt_msgs::Hunt ScavengerRosInterface::get_hunt(std::string hunt_name){

	scavenger_hunt_msgs::GetHunt get_hunt;
	get_hunt.request.hunt_name = hunt_name;
	bool success = _get_hunt_client.call(get_hunt);
	if (!success){
		ROS_ERROR("Call to GetHunt service failed.");
		return scavenger_hunt_msgs::Hunt();
	}
	return get_hunt.response.hunt;	
}

std::vector<scavenger_hunt_msgs::Task> ScavengerRosInterface::get_tasks(std::string hunt_name){
	scavenger_hunt_msgs::Hunt hunt = get_hunt(hunt_name);
	return hunt.tasks;
}

unsigned char ScavengerRosInterface::get_hunt_status(unsigned long proof_id){
	scavenger_hunt_msgs::GetProofStatus get_status;
	get_status.request.id = proof_id;

	_get_status_client.call(get_status);
	// 0 -> Proof is not yet validated
	// 1 -> Proof was marked correct
	// 2 -> Proof was marked incorrect
	return get_status.response.status;
}

bool ScavengerRosInterface::is_hunt_validated(unsigned long proof_id){
	return get_hunt_status(proof_id) != '0';
}

bool ScavengerRosInterface::is_hunt_correct(unsigned long proof_id){
	return get_hunt_status(proof_id) == '1';
}

bool ScavengerRosInterface::is_hunt_incorrect(unsigned long proof_id){
	return get_hunt_status(proof_id) == '2';
}

bool ScavengerRosInterface::is_ok(){
	return _is_ok;
}

/*
 * Send an image proof to the server given the image and the task its associated with
 * Returns true on success, false otherwise.
 */

bool ScavengerRosInterface::upload_proof(scavenger_hunt_msgs::Task task, sensor_msgs::Image image, ros::Time start_time){
	scavenger_hunt_msgs::Proof proof;
	proof.image = image;
	proof.task_duration = ros::Time::now().toSec() - start_time.toSec();
	scavenger_hunt_msgs::SendProof send_proof;
	send_proof.request.proof = proof;
	send_proof.request.task = task;

	bool success = _send_proof_client.call(send_proof);
	if (!success){
		ROS_ERROR("Call to UploadProof service failed. Proof not uploaded.");
		return false;
	}
	unsigned long proof_id = send_proof.response.id;
	if (proof_id == -1){
		ROS_ERROR("Upload request to API failed. Proof not uploaded.");
		return false;
	}
	return true;
}

/*
 * Send a video proof to the server given the path the .mp4 file and the task its associated with
 */

bool ScavengerRosInterface::upload_proof(scavenger_hunt_msgs::Task task, std::string file_path, ros::Time start_time){
	if (!file_accessible(file_path)){
		ROS_ERROR("File path %s is not accessible. Abortting attempt to upload proof.", file_path.c_str());
		return false;
	}
	
	scavenger_hunt_msgs::Proof proof;
	proof.type = scavenger_hunt_msgs::Proof::TYPE_VIDEO;
	proof.file_path = file_path;
	proof.task_duration = ros::Time::now().toSec() - start_time.toSec();
	scavenger_hunt_msgs::SendProof send_proof;
	send_proof.request.proof = proof;
	send_proof.request.task = task;

	bool success = _send_proof_client.call(send_proof);
	if (!success){
		ROS_ERROR("Call to UploadProof service failed. Proof not uploaded.");
		return false;
	}
	unsigned long proof_id = send_proof.response.id;
	if (proof_id == -1){
		ROS_ERROR("Upload request to API failed. Proof not uploaded.");
		return false;
	}
	return true;
}
