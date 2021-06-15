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
		bool _is_ok;
		ros::NodeHandle _nh;
		ros::ServiceClient _get_hunt_client;
		ros::ServiceClient _get_status_client;
		ros::ServiceClient _send_proof_client;

		unsigned char get_hunt_status(unsigned long proof_id);
		bool file_accessible(std::string file_path){
			struct stat buffer;
  			return (stat (file_path.c_str(), &buffer) == 0);
		}
	public:
		ScavengerRosInterface(ros::NodeHandle nh, 
				std::string get_hunt_srv="/scavenger_hunt/get_hunt", 
				std::string send_proof_srv="/scavenger_hunt/send_proof", 
				std::string get_proof_srv="/scavenger_hunt/get_proof_status");
		scavenger_hunt_msgs::Hunt get_hunt(std::string hunt_name);
		std::vector<scavenger_hunt_msgs::Task> get_tasks(std::string hunt_name);
		bool is_hunt_validated(unsigned long proof_id);
		bool is_hunt_correct(unsigned long proof_id);
		bool is_hunt_incorrect(unsigned long proof_id);
		bool is_ok();
		bool upload_proof(scavenger_hunt_msgs::Task task, sensor_msgs::Image image, ros::Time start_time);
		bool upload_proof(scavenger_hunt_msgs::Task task, std::string file_path, ros::Time start_time);
};
