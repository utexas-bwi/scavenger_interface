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

class Hunter {
public:
	/*
	 * General perform tasks 
	 */
	virtual void performHunt() const = 0;
	virtual void performTask() const = 0;

	/*
	 * Find an object
	 */
	virtual void performFindTask(){}

	/*
	 * Go to a location
	 */
	virtual void performLocationTask(){}

	/*
	 * Travel more than a certain distance
	 */
	virtual void performTravelTask(){}

};
