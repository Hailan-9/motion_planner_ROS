/**
 * @copyright Copyright (c) 2023-hailan
 *          jps算法是针对astar算法的改进
 * @file    jps.hpp
 * @brief   使用继承技术，来实现jps，因为jps和astar除了扩展邻居不一样，其他地方完全一样。
 *          使用继承，可以增加代码的复用性！！！！！！
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-17
 */
#pragma once

#include "astar_gridmap_searcher/astar.hpp"

using namespace std;
namespace Grid_Path_Search
{

    // Search and prune neighbors for JPS 2D 修剪邻居
    // JPS_utilities 
    // struct JPS2DNeighbor
    struct JPS2DNeighb
    {
        // 默认共有
            // for each (dx,dy) these contain:  
            // ******相对位置******
            //    ns: neighbors that are always added
            //    f1: forced neighbors to check 检查相关位置是否有障碍物 有则存在强制邻居
            //    f2: neighbors to add if f1 is forced 存强制邻居的相对位置
            // 9：x、y -1~1    2：x y    8：natural neighbors最多八个
            int ns[9][2][8];
            int f1[9][2][2];
            int f2[9][2][2];

            // 二维数组的index，分别代表运动方向的曼哈顿距离0 1 2 
            // ---即起点处没有运动方向、直线运动方向、对角线运动方向
            // 值分别表示natural neighbor-即必定被检查的邻居个数
            // 最多可能的forced neighbor个数
            static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3,2}};
            JPS2DNeighb();
            void print();
        private:
            // natural neighbor
            void Neighb(int dx, int dy, int norm1, int dev, int& tx, int& ty);
            // forced neighbor
            void FNeighb(int dx, int dy, int norm1, int dev,
                 int& fx, int& fy, int& nx, int& ny);
    };



    class JPS : public Astar
    {
        public:
            JPS();
            int jpsSearchPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner);
            
        protected:
            bool isOccupied(const int& x, const int& y);
            bool isFree(const int& x, const int& y);
            // 0 free 100 obstacle -1 unkonwn
            int free_flag = 0;
            bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);
            void getJpsSucc(const Point* curnode, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
            bool hasForced(int x, int y, int dx, int dy);
            void jpsGetJumpPoint(PointPtr curnode, std::vector<Point>& jumpPoints);
            // 智能指针，开辟的内存，不需要自己手动释放，程序会自动释放
            std::shared_ptr<JPS2DNeighb> jps2D;
            Eigen::Vector2f goal_point;
    };





}
