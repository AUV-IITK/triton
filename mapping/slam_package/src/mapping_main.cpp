#include "mapping_main.h"
#include <string>
#include <math.h>
#include <chrono>
#include <numeric>
#include <fstream>


namespace mapping {

void MappingNode::transform_broadcaster_initial(tf::TransformBroadcaster& map_to_odom_broadcaster)
{
	ROS_INFO("Publishing a transform");
	geometry_msgs::TransformStamped map_to_odom_trans;
	map_to_odom_trans.header.stamp = ros::Time::now();
	map_to_odom_trans.header.frame_id = "map";
	map_to_odom_trans.child_frame_id = "odom";

	map_to_odom_trans.transform.translation.x = 0;
	map_to_odom_trans.transform.translation.y = 0;
	map_to_odom_trans.transform.translation.z = 0;
	map_to_odom_trans.transform.rotation.w = 1;
	map_to_odom_trans.transform.rotation.x = 0;
	map_to_odom_trans.transform.rotation.y = 0;
	map_to_odom_trans.transform.rotation.z = 0;

	// send the transform
	map_to_odom_broadcaster.sendTransform(map_to_odom_trans);
	ROS_INFO("Published the transform");
}

MappingNode::MappingNode(const ros::NodeHandlePtr &nh) : nh_(nh), _map( {"elevation", "occupancy"})
{
	_map.setFrameId("map");
	_map.setGeometry(grid_map::Length(20, 20), 1);

	map_publisher = nh_->advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
	vision_subscriber = nh_->subscribe("vision/buoy", 10, &MappingNode::landmark_update_cb, this);
	odom_subscriber = nh_->subscribe("/odometry/filtered", 10, &MappingNode::odometry_update_cb, this);
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", _map.getLength().x(), _map.getLength().y(), _map.getSize()(0), _map.getSize()(1));

	for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it) {
		grid_map::Position position;
		_map.getPosition(*it, position);
		_map.at("elevation", *it) = 0.5;
		_map.at("occupancy", *it) = 0.5;
	}
	loadMapFromYAML(_map);
	particles_initialize();
	//PublishMap();
}

void MappingNode::odometry_update_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	grid_map::Position checked_space(msg->pose.pose.position.x, msg->pose.pose.position.y);

	//need to check if already visited, or object is present here
	if (_map.atPosition("occupancy", checked_space) == 0.5)
		_map.atPosition("occupancy", checked_space) = 0;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	gamma = msg -> pose.pose.orientation.z;
	ROS_INFO("We have already gone to: %f %f at angle %f", x, y, gamma);
	PublishMap();
}

void MappingNode::landmark_update_cb(const geometry_msgs::PoseArray::ConstPtr& msgp)
{
	ROS_INFO("Pose message size %i", msgp->poses.size());
	if (msgp->poses[0].position.x != -1 || msgp->poses[0].position.y != -1 || msgp->poses[0].orientation.z != -1)
	{
		for (int i = 0; i < LANDMARK_ESTIMATION_PARTICLES; i++)
		{
			redlandmarkParticles[i].x = x + msgp->poses[0].position.x;
			redlandmarkParticles[i].y = y + msgp->poses[0].position.y;
		}
	}

	if (msgp->poses[1].position.x != -1 || msgp->poses[1].position.y != -1 || msgp->poses[1].orientation.z != -1)
	{
		for (int i = 0; i < LANDMARK_ESTIMATION_PARTICLES; i++)
		{
			bluelandmarkParticles[i].x = x + msgp->poses[1].position.x;
			bluelandmarkParticles[i].y = y + msgp->poses[1].position.y;
		}
	}

	if (msgp->poses[2].position.x != -1 || msgp->poses[2].position.y != -1 || msgp->poses[2].orientation.z != -1)
	{
		for (int i = 0; i < LANDMARK_ESTIMATION_PARTICLES; i++)
		{
			redlandmarkParticles[i].x = x + msgp->poses[2].position.x;
			redlandmarkParticles[i].y = y + msgp->poses[2].position.y;
		}
	}

	update_map();
}
void MappingNode::loadMapFromYAML(grid_map::GridMap &map)
{
	YAML::Node yaml_map = YAML::LoadFile(map_yaml_location);
	ROS_INFO("Loaded: %d", yaml_map["map"]["gate"]["x"].as<int>());
	grid_map::Position gate_position(yaml_map["map"]["gate"]["x"].as<int>(), yaml_map["map"]["gate"]["y"].as<int>());
	grid_map::Position torpedo_position(yaml_map["map"]["torpedo"]["x"].as<int>(), yaml_map["map"]["torpedo"]["y"].as<int>());
	_map.atPosition("occupancy", gate_position) = 1;
	_map.atPosition("occupancy", torpedo_position) = 1;
	ROS_INFO("Loaded map file");
}
void MappingNode::update_map()
{
	// YAML::Node yaml_map = YAML::LoadFile(map_yaml_location);
	// ROS_INFO("Loaded: %d", yaml_map["map"]["gate"]["x"].as<int>());
	float red_x = 0, red_y = 0, blue_x = 0, blue_y = 0, green_x = 0, green_y = 0;
	for (int i = 0; i < LANDMARK_ESTIMATION_PARTICLES; i++)
	{
		red_x += redlandmarkParticleWeights[i] * redlandmarkParticles[i].x;
		red_y += redlandmarkParticleWeights[i] * redlandmarkParticles[i].y;
		blue_x += bluelandmarkParticleWeights[i] * bluelandmarkParticles[i].x;
		blue_y += bluelandmarkParticleWeights[i] * bluelandmarkParticles[i].y;
		green_x += greenlandmarkParticleWeights[i] * greenlandmarkParticles[i].x;
		green_y += greenlandmarkParticleWeights[i] * greenlandmarkParticles[i].y;

	}
	grid_map::Position red_buoy_position((int)red_x, (int)red_y);
	grid_map::Position blue_buoy_position((int)blue_x, (int)blue_y);
	grid_map::Position green_buoy_position((int)green_x, (int)green_y);
	_map.atPosition("occupancy", red_buoy_position) = 2;
	_map.atPosition("occupancy", blue_buoy_position) = 2;
	_map.atPosition("occupancy", green_buoy_position) = 2;
	ROS_INFO("Map updated");
}
void MappingNode::PublishMap()
{
	ros::Time time = ros::Time::now();
	// Publish grid map.
	_map.setTimestamp(time.toNSec());
	//ROS_INFO("Position %f", _map.getPosition().x());
	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(_map, message);
	message.info.pose.position.x = x;
	message.info.pose.position.y = y;
	//std::string str[]= {"occupancy", "elevation"};
	message.layers[0] = "occupancy";
	message.layers[0] = "elevation";

	map_publisher.publish(message);
	ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
	// Wait for next cycle.
}

void MappingNode::Spin()
{
	ros::Rate loop_rate(15); // 100 hz
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void MappingNode::particles_initialize()
{
	ROS_INFO("initializing the particles");
	for (int i = 0; i < ROBOT_ESTIMATION_PARTICLES; i++)
	{
		robotPoseParticles[i].x = START_X;
		robotPoseParticles[i].y = START_Y;
		robotPoseParticles[i].theta = START_TH;

		robotParticleWeights[i] = 1.00 / (float)ROBOT_ESTIMATION_PARTICLES;

	}

	for (int i = 0; i < LANDMARK_ESTIMATION_PARTICLES; i++)
	{
		redlandmarkParticles[i].x = RED_X;
		redlandmarkParticles[i].y = RED_Y;
		redlandmarkParticles[i].theta = RED_TH;
		redlandmarkParticleWeights[i] = 1.00 / (float) LANDMARK_ESTIMATION_PARTICLES;


		bluelandmarkParticles[i].x = BLUE_X;
		bluelandmarkParticles[i].y = BLUE_Y;
		bluelandmarkParticles[i].theta = BLUE_TH;
		bluelandmarkParticleWeights[i] = 1.00 / (float) LANDMARK_ESTIMATION_PARTICLES;

		greenlandmarkParticles[i].x = GREEN_X;
		greenlandmarkParticles[i].y = GREEN_Y;
		greenlandmarkParticles[i].theta = GREEN_TH;
		greenlandmarkParticleWeights[i] = 1.00 / (float) LANDMARK_ESTIMATION_PARTICLES;

	}

}


}