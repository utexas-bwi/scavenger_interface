/*
 * An abstract scavenger Hunter class. This outlines the potential cabilities of a robot 
 * that performs Hunts from https://scavenger-hunt.cs.utexas.edu
 * 
 * The non-pure abstract functions are more informative than functional. More of these
 * abstract task functions may be added as possible tasks are added on the Scavenger website.
 *
 * Author Maxwell Svetlik, 2021
 */

#pragma once
#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/Task.h>
#include <sensor_msgs/Image.h>

class Hunter {
public:
	/*
	 * General perform tasks 
	 */
	virtual void prepare_hunt(scavenger_hunt_msgs::Hunt) const = 0;
	virtual sensor_msgs::Image perform_image_task(scavenger_hunt_msgs::Task) const = 0;
	virtual std::string perform_video_task(scavenger_hunt_msgs::Task) const = 0;

	/*
	 * Find an object
	 */
	virtual void perform_find_task(){}

	/*
	 * Go to a location
	 */
	virtual void perform_location_task(){}

	/*
	 * Travel more than a certain distance
	 */
	virtual void perform_travel_task(){}

};
