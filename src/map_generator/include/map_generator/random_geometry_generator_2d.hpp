/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    random_geometry_generator2d.h
 * @brief   描述 遵循大小驼峰编码规范
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-03
 */

#include <nav_msgs/GridCells.h>
#include "map_generator/type.hpp"

class RandomGeometryGenerator2D
{
    public:
        RandomGeometryGenerator2D(float p_x_lower, float p_x_upper, float p_y_lower,
                                  float p_y_upper, float p_obs_w_lower, float p_obs_w_upper,
                                  float p_map_init_x, float p_map_init_y, float p_resolution);
        // 常量成员函数 因为最后加了const
        PointCloudPtr obstacle(unsigned int num_obstacle) const;
        nav_msgs::GridCells generateGridCells(const PointCloudPtr &point_clound_ptr);
        PointXYZ coordinate2GridIndex(const PointXYZ &pt);
        PointXYZ gridIndex2Coordinate(const PointXYZ &pt);

    private:
        // 上下界的长度 单位m
        float x_lower, x_upper;
        float y_lower, y_upper;
        float obstacle_width_lower, obstacle_width_upper;
        float map_init_x, map_init_y;
        // 分辨率，m/grid
        float resolution;
        // 栅格个数
        int GRID_X_SIZE;
        int GRID_Y_SIZE;

};