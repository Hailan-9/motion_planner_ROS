/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    random_geometry_generator2d.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-03
 */

#include "map_generator/random_geometry_generator_2d.hpp"

#include <random>
using namespace std;

RandomGeometryGenerator2D::RandomGeometryGenerator2D(
    float p_x_lower, float p_x_upper, float p_y_lower,
    float p_y_upper, float p_obs_w_lower, float p_obs_w_upper,
    float p_map_init_x, float p_map_init_y, float p_resolution)
    :x_lower(p_x_lower), x_upper(p_x_upper), y_lower(p_y_lower),
    y_upper(p_y_upper), obstacle_width_lower(p_obs_w_lower),
    obstacle_width_upper(p_obs_w_upper), map_init_x(p_map_init_x),
    map_init_y(p_map_init_y), resolution(p_resolution)
{
    GRID_X_SIZE = int((x_upper - x_lower) / resolution);
    GRID_Y_SIZE = int((y_upper - y_lower) / resolution);
}


PointCloudPtr RandomGeometryGenerator2D::obstacle(unsigned int num_obstacle) const
{
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::random_device random_device;
    std::mt19937 gen(random_device());

    std::uniform_real_distribution<float> rand_x(x_lower, x_upper);
    std::uniform_real_distribution<float> rand_y(y_lower, y_upper);
    std::uniform_real_distribution<float> rand_w(obstacle_width_lower, 
                                                obstacle_width_upper);
    
    PointXYZ point_xyz;
    for (unsigned int i = 0; i < num_obstacle; ++i)
    {
        float x, y, w_x, w_y;
        x = rand_x(gen);
        y = rand_y(gen);
        w_x = rand_w(gen);
        w_y = rand_w(gen);

        if (std::sqrt( std::pow(x - map_init_x, 2) + std::pow(y - map_init_y, 2)) < 0.5f)
        {
            continue;
        }

        x = std::floor(x / resolution) * resolution + resolution * 0.5f;
        y = std::floor(y / resolution) * resolution + resolution * 0.5f;

        float w_x_number = std::ceil(w_x / resolution);
        float w_y_number = std::ceil(w_y / resolution); 

        for (float d_x = -w_x_number*0.5f; d_x < w_x_number*0.5f;)
        {
            for (float d_y = -w_y_number*0.5f; d_y < w_y_number*0.5f;)
            {
                point_xyz.x = x + d_x*resolution + 0.001f;
                point_xyz.y = y + d_y*resolution + 0.001f;
                point_cloud_ptr->points.emplace_back(point_xyz);
                d_y += 1.0f;
            }
            d_x += 1.0f;
        }
    }

    point_cloud_ptr->width = point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    point_cloud_ptr->is_dense = true;

    return point_cloud_ptr;
}


nav_msgs::GridCells RandomGeometryGenerator2D::generateGridCells(const PointCloudPtr& point_cloud_ptr)
{
    nav_msgs::GridCells grid_cells;
    grid_cells.header.frame_id = "world";
    grid_cells.cell_height = resolution;
    grid_cells.cell_width = resolution;

    geometry_msgs::Point obstacle;

    for (const auto &point : *point_cloud_ptr)
    {
        PointXYZ point_xyz;
        point_xyz = gridIndex2Coordinate(coordinate2GridIndex(point));

        obstacle.x = point_xyz.x;
        obstacle.y = point_xyz.y;
        obstacle.z = 0.0f;

        grid_cells.cells.emplace_back(obstacle);

    }

    return grid_cells;
}


PointXYZ RandomGeometryGenerator2D::coordinate2GridIndex(const PointXYZ &pt)
{
    PointXYZ grid_point;

    grid_point.x = float(std::min(std::max(int((pt.x - x_lower) / resolution), 0), GRID_X_SIZE - 1));
    grid_point.x = float(std::min(std::max(int((pt.x - x_lower) / resolution), 0), GRID_X_SIZE - 1));

    return grid_point;
}

PointXYZ RandomGeometryGenerator2D::gridIndex2Coordinate(const PointXYZ& pt)
{
    PointXYZ coordinate_point;

    coordinate_point.x = (pt.x + 0.5f) * resolution + x_lower;
    coordinate_point.y = (pt.y + 0.5f) * resolution + y_lower;

    return coordinate_point;
}