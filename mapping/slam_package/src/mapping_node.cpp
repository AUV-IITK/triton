#include <ros/ros.h>
#include "mapping_main.h"
// #include "slam_spine.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "mapping_node");

	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
	mapping::MappingNode mapping_node(nh);
	ROS_INFO("Started the mapping_node handler");
	// fastslam::SlamNode slam_node(nh);
	// ROS_INFO("Started the slam_node handler");
	// slam_node.Spin();
	mapping_node.Spin();
}