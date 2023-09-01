/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    sample_path_finding.hpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-28
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

// 用于订阅rviz里面的起点 终点
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <ctime>

// namespace

// 地图信息

// 单位 cells 也就是格子数
// int width = 0;
// int height = 0;
Eigen::Vector2f map_size;
Eigen::Vector2f map_origin;
// 单位 m/cell
float resolution = 0.0;

// 地图数据使用二维数组来表示
std::vector<std::vector<int>> mapData;
// 算法的起点 终点 在世界地图下的 map
Eigen::Vector2f startPoint;
Eigen::Vector2f goalPoint;

// 世界地图坐标系下
std::vector<Eigen::Vector2f> pathPoint;

ros::Publisher rrt_PathPub;
ros::Publisher rrt_Pub_Startpoint;
ros::Publisher rrt_Pub_Goalpoint;

ros::Subscriber rrt_MapSub;
ros::Subscriber initPoseSub;
ros::Subscriber goalPoseSub;


void startFindPath(ros::NodeHandle& nh);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgPtr);

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgPtr);
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgPtr, ros::NodeHandle& nh);

void publishPath();



