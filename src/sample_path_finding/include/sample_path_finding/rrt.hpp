/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    rrt.hpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-26
 */
#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <random>
#include <cmath>

// 使用节点数据类型
#include "astar_gridmap_searcher/astar.hpp"



using namespace Grid_Path_Search;
using namespace std;
using namespace Eigen;

// namespace 

class RRT
{
    public:
        RRT() {};// 构造函数声明和定义，空定义
        ~RRT();// 析构函数声明
        void init(const vector<vector<int>> &_map, Eigen::Vector2f origin);
        int pathFind(const Point& startPoint, const Point& endPoint);
        vector<Eigen::Vector2f> getPath();
        void setParam(ros::NodeHandle& nh);
        void reset();

    protected:
        void retrievePath(PointPtr end_node);
        Point randomSample();
        PointPtr getNearestNode(Point x_rand);
        Point steer(Point x_rand, PointPtr x_nearest);
        // edge();
        bool collisionFree(Point x_new, PointPtr x_nearest);


        // 函数声明
        // 世界坐标系（也就是地图坐标系）-->栅格地图坐标系
        Eigen::Vector2f world2Gridmap(float w_x, float w_y);
        // 栅格地图坐标系-->世界坐标系（也就是地图坐标系）
        Eigen::Vector2f gridmap2World(float gm_x, float gm_y);


        /* ---------数据---------*/
        // 步长
        float stepSize;
        float goal_tolerance;
        // 分配的节点内存个数
        int allocate_num;
        // 分次进行递进检查x_new和x_nearest的连线是否碰到了障碍物
        float collision_check_num;

        int use_node_num;
        // 装载collision free的新节点
        std::vector<PointPtr> node_list;
        vector<PointPtr> path_nodes;

        // 采样点的范围
        Eigen::Vector2f min_val;
        Eigen::Vector2f max_val;

        // 路径搜索过程中的节点资源池，里面存放了分配数量的指向节点类型的指针，使用new开辟的
        // 若是搜索过程中，用的节点数量超过了资源池的容量，那么就是run out of memory
        vector<PointPtr> path_node_pool;


        /* ---------参数--------- */
        // 地图信息
        // 栅格地图分辨率 单位 m/cell
        float resolution;
        // 单位 cells 也就是格子数 宽 高
        Eigen::Vector2f map_size;
        // 单位 m
        // 栅格地图坐标系原点相对于世界坐标系原点的位置 x y
        Eigen::Vector2f map_origin;
        std::vector<std::vector<int>> map;

        // 可视化
        // 发布树结构
        ros::Publisher tree_pub;
        ros::Publisher path_pub;
        visualization_msgs::Marker tree;
        nav_msgs::Path path;

};