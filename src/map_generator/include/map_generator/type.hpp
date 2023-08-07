/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    type.h
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-03
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

typedef typename pcl::PointXYZ PointXYZ;
typedef typename pcl::PointCloud<PointXYZ> PointCloud;
typedef typename pcl::PointCloud<PointXYZ>::Ptr PointCloudPtr;

// Eigen::aligned_allocator<Eigen::Vector3f> 是用于在Eigen库中指定特定内存对齐方式的分配器（allocator）。
// 它与标准的C++ STL分配器（如std::allocator）有所不同，因为Eigen库在处理数据时，需要特定的内存对齐以提高性能。
typedef typename std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVec3f;