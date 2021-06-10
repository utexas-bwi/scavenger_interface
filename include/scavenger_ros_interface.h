/*
 * A ROS library that wraps around the Scavenger Hunt API to simplify common interactions 
 * such as fetching tasks and uploading proofs.
 * 
 * Author Maxwell Svetlik, 2021
 */

#include <ros/ros.h>
#include <ros/time.h>
#include <scavenger_hunt_msgs/GetProofStatus.h>
#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/Task.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <sys/stat.h>
#include <vector>

class ScavengerRosInterface {
	private: 
		bool _isOk;
		ros::NodeHandle _nh;
		ros::ServiceClient _getHuntClient;
		ros::ServiceClient _getStatusClient;
		ros::ServiceClient _sendProofClient;

		unsigned char getHuntStatus(unsigned long proofId);
		bool fileAccessible(std::string filePath){
			struct stat buffer;
  			return (stat (filePath.c_str(), &buffer) == 0);
		}
	public:
		ScavengerRosInterface(ros::NodeHandle nh, 
				std::string getHuntSrv="/scavenger_hunt/get_hunt", 
				std::string sendProofSrv="/scavenger_hunt/send_proof", 
				std::string getProofSrv="/scavenger_hunt/get_proof_status");
		scavenger_hunt_msgs::Hunt getHunt(std::string huntName);
		std::vector<scavenger_hunt_msgs::Task> getTasks(std::string huntName);
		bool isHuntValidated(unsigned long proofId);
		bool isHuntCorrect(unsigned long proofId);
		bool isHuntIncorrect(unsigned long proofId);
		bool isOk();
		bool uploadProof(scavenger_hunt_msgs::Task task, sensor_msgs::Image image, ros::Time startTime);
		bool uploadProof(scavenger_hunt_msgs::Task task, std::string filePath, ros::Time startTime);
};
