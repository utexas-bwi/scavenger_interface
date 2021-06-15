# scavenger_interface
An interface to simplify connecting and interacting with the Scavenger Hunt Challenge : https://scavenger-hunt.cs.utexas.edu

# Installation
This repo depends on the [scavenger_hunt_api](https://github.com/utexas-bwi/scavenger_hunt_api) repository. 
In the src of a catkin workspace, 

`$ git clone https://github.com/utexas-bwi/scavenger_hunt_api.git --recursive`

`$ git clone https://github.com/utexas-bwi/scavenger_interface.git`

## Building
Building should be as simple as invoking catkin tools in the root of your workspace.

`$ catkin build`

## Running
### Pre-running
Follow the instructions on the [scavenger_hunt_api README](https://github.com/utexas-bwi/scavenger_hunt_api/blob/master/README.md) for creating an account, and changing the config file with your credentials.

Change the perception topics in `scavenger_interface.launch` if necessary. The `image_topic` and `image_detection_topic` should be the topics of an RGB image stream from a camera, and a image recognition stream, respectively. Currently they're configured for the default values for [darknet_ros](https://github.com/leggedrobotics/darknet_ros).

### Launching a Passive Hunter
To run, you'll need to bring up your robot and perception systems as usual. This may be base drivers & YOLO, etc.

E.g. `$ roslaunch darknet_ros darknet_ros.launch`

Then bring up the passive hunter launch file: 

`$ roslaunch scavenger_interface scavenger_interface.launch hunt_name:="My Hunt"`

Where `hunt_name` should be specified by you.
By default the `hunter_launcher` is configured to bring up a Passive Hunter.

## Code Overview
This package contains the following primary components:

- `scavenger_ros_interface` : a ROS-dependent library that directly interacts with the API and provides simple ways to {get a hunt, retrieve tasks, submit proofs} via the API
- `include/hunter.h` : an abstract class that serves as a template for a custom Hunter implentation for your own robot platform
- `passive_hunter` : an implementation of a scavenger Hunter that attempts to fufill scavenger tasks passively, i.e. while carrying out another primary function
- `hunter_launcher` : this is the launchpad for a scavenger Hunter. It utilizes the `hunter.h` form to query a Hunter in a generic way.

### Making Changes
Creating your own Hunter is as simple as creating a new class that implements the `hunter.h` abstract methods, and changing the type of Hunter that gets created in `hunter_launcher`

Feel free to create new launch files as appropriate.

