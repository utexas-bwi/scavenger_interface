/* 
 * An example of a Scavenger Hunter that runs passively, without strictly controlling 
 * the robot's actions to perform or complete tasks. Thus, it is the easiest to get 
 * running on a new robot platform, but may not be high scoring, depending on the 
 * current actions of the robot.
 *
 * Author Maxwell Svetlik, 2021
 *
 */

#include <ros/ros.h>

#include "passive_hunter.h"


PassiveHunter::PassiveHunter() : Hunter(){}

void PassiveHunter::performHunt() const {

}

void PassiveHunter::performTask() const {

}
