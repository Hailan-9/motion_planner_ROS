/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    map_generator2d.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-04
 */

#include "map_generator/random_geometry_generator_2d.hpp"

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "map_generator2d");
    ros::NodeHandle nh("~");


    ros::Publisher global_pointcloud_map_publisher = 
        nh.advertise<sensor_msgs::PointCloud2>("global_pointcloud_map", 10);
    ros::Publisher global_grid_cells_publisher = 
        nh.advertise<nav_msgs::GridCells>("global_grid_cells_map", 10);
    // 参数服务器获取参数
    const unsigned int number_obstacle = nh.param("map/obstacle_number", 100);
    const float resolution = nh.param("map/resolution", 0.05f);
    const float sensor_rate = nh.param("map/sensor_rate", 0.5f);

    const float x_size = nh.param("map/x_size", 10.0f);
    const float y_size = nh.param("map/y_size", 10.0f);
    const float init_x = nh.param("map/init_x", 0.0f);
    const float init_y = nh.param("map/init_y", 0.0f);

    const float obstacle_w_lower = nh.param("map/obstacle_w_lower", 0.3f);
    const float obstacle_w_upper = nh.param("map/obstacle_w_upper", 0.0f);

    float x_lower = -x_size * 0.5f;
    float x_upper = x_size * 0.5f;
    float y_lower = -y_size * 0.5f;
    float y_upper = y_size * 0.5f;

    RandomGeometryGenerator2D random_geometry_generator2d(
        x_lower, x_upper, y_lower, y_upper, obstacle_w_lower,
        obstacle_w_upper, init_x, init_y, resolution);

    PointCloudPtr point_cloud_ptr(new PointCloud);
    point_cloud_ptr = random_geometry_generator2d.obstacle(number_obstacle);

    nav_msgs::GridCells grid_cells = random_geometry_generator2d.generateGridCells(point_cloud_ptr);

    sensor_msgs::PointCloud2 global_pointcloud_ros;
    pcl::toROSMsg(*point_cloud_ptr, global_pointcloud_ros);

    global_pointcloud_ros.header.frame_id = "world";
    // 当前时间戳
    global_pointcloud_ros.header.stamp = ros::Time::now();

    grid_cells.header.stamp = ros::Time::now();
    ros::Rate rate(sensor_rate);

    ROS_INFO("start----------");
    while (ros::ok())
    {
        global_pointcloud_map_publisher.publish(global_pointcloud_ros);
        global_grid_cells_publisher.publish(grid_cells);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
