/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    a_star_gridmap_search.hpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-05
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

// 单位 m
// 栅格地图坐标系原点相对于世界坐标系原点的位置
float origin_x = 0.0;
float origin_y = 0.0;
// 单位 cells 也就是格子数
// int width = 0;
// int height = 0;
Eigen::Vector2f map_size;
Eigen::Vector2f map_origin;
// 单位 m/cell
float resolution = 0.0;

// 地图数据使用二维数组来表示
std::vector<std::vector<int>> mapData;
// astar算法的起点 终点 在世界地图下的 map
Eigen::Vector2f startPoint;
Eigen::Vector2f goalPoint;
// 世界地图坐标系下
std::vector<Eigen::Vector2f> pathPoint;

ros::Publisher astar_PathPub;
ros::Publisher astar_Pub_Startpoint;
ros::Publisher astar_Pub_Goalpoint;

ros::Subscriber astar_MapSub;
ros::Subscriber initPoseSub;
ros::Subscriber goalPoseSub;


void startFindPath();

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgPtr);

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgPtr);
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgPtr, ros::NodeHandle& nh);

void publishPath();



