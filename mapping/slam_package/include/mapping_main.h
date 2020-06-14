#ifndef MAPPING_MAIN_H_
#define MAPPING_MAIN_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include "nav_msgs/Odometry.h"
#include "conf.h"
#include "util.h"
#include <bits/stdc++.h>
#include "geometry_msgs/PoseArray.h"
#include "ekf.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <iostream>

namespace mapping
{

class MappingNode
{
public:
    MappingNode(const ros::NodeHandlePtr &nh);

    void transform_broadcaster_initial(tf::TransformBroadcaster& map_to_odom_broadcaster);
    void odometry_update_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void landmark_update_cb(const geometry_msgs::PoseArray::ConstPtr& msgp);
    void loadMapFromYAML(grid_map::GridMap &map);
    void PublishMap();
    void Spin();
    void particles_initialize();
    void update_map();

private:
    struct Pose {
        float x, y, theta;
        mat3 cov;
    };

    ros::NodeHandlePtr nh_;

    ros::Subscriber odom_subscriber;
    ros::Publisher map_publisher;
    ros::Subscriber vision_subscriber;
    grid_map::GridMap _map;
    float x;
    float y;
    float gamma;
    tf::TransformBroadcaster map_to_odom_broadcaster;

    struct Pose robotPoseParticles[ROBOT_ESTIMATION_PARTICLES];
    float robotParticleWeights[ROBOT_ESTIMATION_PARTICLES];

    struct Pose redlandmarkParticles[LANDMARK_ESTIMATION_PARTICLES];
    float redlandmarkParticleWeights[LANDMARK_ESTIMATION_PARTICLES];

    struct Pose bluelandmarkParticles[LANDMARK_ESTIMATION_PARTICLES];
    float bluelandmarkParticleWeights[LANDMARK_ESTIMATION_PARTICLES];

    struct Pose greenlandmarkParticles[LANDMARK_ESTIMATION_PARTICLES];
    float greenlandmarkParticleWeights[LANDMARK_ESTIMATION_PARTICLES];


    std::string map_yaml_location = "/home/sskorde/auv_ws/src/navigation_layer/mapping/config/pool.yaml";

};

class SlamParticle {
public:
    float weight_;

private:
    vec3 position_;
    vec3 orientation_;

    std::default_random_engine gen_;
    std::normal_distribution<float> uniform_;
    std::normal_distribution<float> gaussian_;

    std::unordered_map<std::string, SlamEKF> landmark_filters_;

public:
    SlamParticle(float weight, const vec3 &pos, const vec3 &ori);
    void AddLandmark(std::string id, const vec3 &relpos, const mat3 &conv);
    void UpdateLandmark(std::string id, const vec3 &relpos);
    void UpdateParticle(const vec6 &u, float weight);
    float UpdateWeight(std::string id, const vec3 &relpos, float certainty);

    void ReinitRandom();

    vec3 GetState();
    vec6 GetState(std::string id);
};

class SlamFilter {
private:
    int num_particles_;
    std::vector<SlamParticle> particles_;
    std::vector<float> weights_;
    std::unordered_set<std::string> landmarks_;

    std::default_random_engine gen_;

public:
    SlamFilter(int n);
    SlamFilter(int n, const vec3 &pos, const vec3 &ori);
    void Update(const vec6 &u);
    void Landmark(std::string id, const vec3 &relpos, const mat3 &cov);

    vec3 GetState();
    vec6 GetState(std::string id);

    void GNUPlotOut();

private:
    void NewLandmark(std::string id, const vec3 &relpos, const mat3 &cov);
    void UpdateLandmark(std::string id, const vec3 &relpos, const mat3 &cov);
    void NormalizeWeights();
    void Resample();

};

} // namespace mapping
#endif //MAPPING_NODE_H_
