/*
 *
 */

#include <ros/ros.h>

#include "passive_hunter.h"
#include "scavenger_ros_interface.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "scavenger_hunter_launcher");
	ros::NodeHandle nh;

	ScavengerRosInterface* sri = new ScavengerRosInterface(nh);
	PassiveHunter* ph = new PassiveHunter();

	ros::spin();

	return 0;
}
