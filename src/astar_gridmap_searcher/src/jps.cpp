/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    jps.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-17
 */

#include "astar_gridmap_searcher/jps.hpp"

using namespace Grid_Path_Search;

/*
jps和A*算法的区别是，jps会向一个方向一直搜索，直到找到一个跳点。jps会将这个跳点加入到openlist。

跳点定义：当前点 x 满足以下三个条件之一：

节点 x 是起点/终点。
节点 x 至少有一个强迫邻居。
如果父节点在斜方向（意味着这是斜向搜索），节点x的水平或垂直方向上有满足条件a，b的点。

refer to：https://blog.csdn.net/qq_40606107/article/details/120789713
*/




JPS::JPS()
{
    // use this function to create shared_ptr
    jps2D = std::make_shared<JPS2DNeighb>();
    jps2D->print();
}




/**
 * @brief   描述
 * 
 * @param   startPoint  参数描述
 * @param   endPoint    参数描述
 * @return  int 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-17
 */
int JPS::jpsSearchPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{

    // 坐标系转换
    startPoint.set_pos(world2Gridmap(startPoint.w_x,startPoint.w_y)(0),
    world2Gridmap(startPoint.w_x,startPoint.w_y)(1));

    endPoint.set_pos(world2Gridmap(endPoint.w_x,endPoint.w_y)(0),
    world2Gridmap(endPoint.w_x,endPoint.w_y)(1));

    goal_point<<endPoint.x, endPoint.y;

    cout <<"gridmap frame---start: "<<startPoint.x<<","<<startPoint.y<<endl;

    cout <<"gridmap frame---goal: "<<endPoint.x<<","<<endPoint.y<<endl;


    geometry_msgs::Point node_closed_point;


    if (isOccupied(startPoint.x, startPoint.y) || isOccupied(endPoint.x, endPoint.y))
    {
        cout <<"init or goal set wrong! in obstacle! again set"<<endl;
        return NO_PATH;
    }

    // 首先是起始点添加进openlist
    Point* curPoint = path_node_pool[0];
    curPoint->x = startPoint.x;
    curPoint->y = startPoint.y;
    curPoint->w_x = gridmap2World(curPoint->x,curPoint->y)(0);
    curPoint->w_y = gridmap2World(curPoint->x,curPoint->y)(1);

    curPoint->pos = startPoint.pos;
    curPoint->set_w_pos(startPoint.w_pos);

    curPoint->G = 0;
    curPoint->H = getH(curPoint,&endPoint);
    curPoint->F = getF(curPoint);
    curPoint->parent = NULL;
    curPoint->point_State = IN_OPEN_SET;
    curPoint->expand_dir = Eigen::Vector2f::Zero();

    openList.push(curPoint);
    use_node_num += 1;
    expanded_nodes.insert(curPoint->pos,curPoint);

    do{

        // // 开放列表中未被访问过的节点的可视化。
        // // 也就是在开放列表中，每次循环中，未被作为f值最小的节点弹出来的。弹出来的就是被扩展了expanded，进closeset中
            
        // // 绿色
        // node_visited_visual.header.frame_id = "map";
        // node_visited_visual.header.stamp = ros::Time::now();
        // node_visited_visual.type = visualization_msgs::Marker::CUBE_LIST;
        // node_visited_visual.action = visualization_msgs::Marker::ADD;
        // node_visited_visual.id = 2;

        // node_visited_visual.pose.orientation.x = 0.0;
        // node_visited_visual.pose.orientation.y = 0.0;
        // node_visited_visual.pose.orientation.z = 0.0;
        // node_visited_visual.pose.orientation.w = 1.0;

        // node_visited_visual.color.a = 1.0;
        // node_visited_visual.color.r = 0.0;
        // node_visited_visual.color.g = 1.0;
        // node_visited_visual.color.b = 0.0;

        // node_visited_visual.scale.x = resolution * 1;
        // node_visited_visual.scale.y = resolution * 1;
        // node_visited_visual.scale.z = resolution * 0.001;

        ros::Duration(1.5).sleep();
        cout <<"@@@@@@@@@@@@@@@@@@"<<endl;
        // 寻找开启列表中F值最低的点，称其为当前点
        curPoint = openList.top();

        /* -------------显示closeset中的节点---这里使用世界地图坐标系下------------- */
        // curPoint->set_w_pos(gridmap2World(curPoint->x, curPoint->y));
        node_closed_point.x = curPoint->w_x + resolution * 0.5;
        node_closed_point.y = curPoint->w_y + resolution * 0.5;

        node_closed_visual.points.emplace_back(node_closed_point);
        node_closed_Pub.publish(node_closed_visual);



        /* -----------------Determine if the destination has been reached--------------------- */
        bool reach_goal = abs(curPoint->x - endPoint.x) <=0 && abs(curPoint->y - endPoint.y) <=0;
        if(reach_goal)
        {
            
            cout <<"path search success!!!!!!!!!"<<endl;
            cout << "use node num: " << use_node_num << endl;
            cout << "iter num: " << iter_num << endl;
            cout <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;

            outfile <<"*****************************************************"<<endl;
            outfile <<"path search success!!!!!!!!!"<<endl;
            outfile <<"weight_g: "<<weight_g <<" weight_h: "<<weight_h<<endl;
            outfile <<"Heuristic_Options: "<<Heuristic_Options<<endl;
            outfile <<"total cost(G): "<<curPoint->G<<endl;
            outfile << "use node num: " << use_node_num << endl;
            outfile << "iter num: " << iter_num << endl;
            outfile <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;

            retrievePath(curPoint);

            return REACH_END;
        }

        openList.pop();
        curPoint->point_State == IN_CLOSE_SET;
        iter_num += 1;


        /* *******************扩展邻居节点，此处是和astar算法的唯一区别******************* */
        // get the succetion 跳点又称感兴趣的点、后继点等
        // JPS算法的核心是找跳点，也就是找特殊的点，作为邻居neighbors,进行加入openlist等操作
        std::vector<Point> jumpPoints;
        jpsGetJumpPoint(curPoint, jumpPoints);

        for (int s = 0; s < (int)jumpPoints.size(); s++)
        {
            Eigen::Vector2f temp_point;
            temp_point <<jumpPoints[s].x, jumpPoints[s].y;
        
            PointPtr neighbor_node = expanded_nodes.find(temp_point);
            
            if (neighbor_node != NULL && neighbor_node->point_State == IN_CLOSE_SET)
            {
                continue;
            }
            // 不在openlist中
            if (neighbor_node == NULL)
            {

                neighbor_node = path_node_pool[use_node_num];
                neighbor_node->parent = curPoint;
                neighbor_node->set_pos(temp_point(0), temp_point(1));
                neighbor_node->G = getG(curPoint,neighbor_node);
                neighbor_node->H = getH(neighbor_node,&endPoint);
                neighbor_node->F = getF(neighbor_node);
                neighbor_node->point_State = IN_OPEN_SET;
                neighbor_node->set_w_pos(gridmap2World(neighbor_node->x, neighbor_node->y));
                // JPS专属 更新其父节点到该节点的扩展方向
                neighbor_node->expand_dir = jumpPoints[s].expand_dir;

                openList.push(neighbor_node);
                expanded_nodes.insert(neighbor_node->pos, neighbor_node);

                use_node_num += 1;

                if (use_node_num == allocate_num)
                {
                    cout <<"***run out of node_pool memory***" <<endl;
                    cout <<"use_node_num: "<<use_node_num<<endl;
                    cout <<"iterm_num: "<<iter_num<<endl;
                    cout <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;
                    
                    outfile <<"*****************************************************"<<endl;
                    outfile <<"***run out of node_pool memory***" <<endl;
                    outfile <<"weight_g: "<<weight_g <<" weight_h: "<<weight_h<<endl;
                    outfile <<"Heuristic_Options: "<<Heuristic_Options<<endl;
                    outfile <<"total cost(G): "<<curPoint->G<<endl;
                    outfile <<"use_node_num: "<<use_node_num<<endl;
                    outfile <<"iterm_num: "<<iter_num<<endl;
                    outfile <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;

                    return NO_PATH;
                }
            }
            else if(neighbor_node->point_State == IN_OPEN_SET)
            {
                if (getG(curPoint, neighbor_node) < neighbor_node->G)
                {   
                    neighbor_node->parent == curPoint;
                    neighbor_node->G = getG(curPoint,neighbor_node);
                    neighbor_node->F = getF(neighbor_node);
                    // // JPS专属 更新其父节点到该节点的扩展方向
                    // neighbor_node->expand_dir = jumpPoints[s].expand_dir;
                }
            }
            else
            {
                continue;
            }
                 
        }
        

    }while(!openList.empty());

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num << endl;
    cout << "iter num: " << iter_num << endl;
    cout <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;
   
    outfile <<"*****************************************************"<<endl;
    outfile << "open set empty, no path!" << endl;
    outfile <<"weight_g: "<<weight_g <<" weight_h: "<<weight_h<<endl;
    outfile <<"Heuristic_Options: "<<Heuristic_Options<<endl;
    outfile <<"total cost(G): "<<curPoint->G<<endl;
    outfile << "use node num: " << use_node_num << endl;
    outfile << "iter num: " << iter_num << endl;
    outfile << "node_closed_num: "<<node_closed_visual.points.size()<<endl;

    return NO_PATH;
}


/**
 * @brief   寻找跳点
 * 
 * @param   curnode     参数描述
 * @param   jumpPoints  搜索到的跳点集合
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-19
 */
void JPS::jpsGetJumpPoint(PointPtr curnode, std::vector<Point>& jumpPoints)
{
    const int norm1 = std::abs(curnode->expand_dir(0)) + std::abs(curnode->expand_dir(1));
    int num_neib = jps2D->nsz[norm1][0];
    int num_fneib = jps2D->nsz[norm1][1];
    int id = 3*(curnode->expand_dir(1) + 1) + curnode->expand_dir(0) + 1;

    // 对当前节点的所有自然节点方向和强迫节点方向进行搜索 two step！
    // no move (norm 0):        8 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          2 forced neighbors to check
    //                          2 neighbors to add if forced

    // 对当前节点的所有---自然节点方向---和---强迫节点方向---进行搜索 two step！
    for ( int dev = 0; dev < num_neib + num_fneib; dev++)
    {
        cout <<"dev-------: "<<dev<<endl;
        int new_x, new_y;
        int dx, dy;
        // 首先检查其必定存在的自然邻居，即先沿着当前节点的父节点指向当前节点的方向进行搜索跳点
        if (dev < num_neib)
        {
            dx = jps2D->ns[id][0][dev];
            dy = jps2D->ns[id][1][dev];
            if(!jump(curnode->x, curnode->y, dx, dy, new_x, new_y))
                continue;
        }
        // 对可能存在的---强迫邻居---进行搜索
        else
        {
            int nx = curnode->x + jps2D->f1[id][0][dev - num_neib];
            int ny = curnode->y + jps2D->f1[id][1][dev - num_neib];
            // 其次，如果当前节点的对应位置有障碍物，说明当前节点可能存在强迫节点，
            // 需要扩展当前节点指向强迫节点的这一方向
            if (isOccupied(nx,ny))
            {
                dx = jps2D->f2[id][0][dev - num_neib];
                dy = jps2D->f2[id][1][dev - num_neib];
                if (!jump(curnode->x, curnode->y, dx, dy, new_x, new_y))
                    continue;
            }
            else
                continue;
        }

        //jump返回false则该搜索方向碰到了障碍物、边界，continue结束本次循环
        //jump返回true则找到了跳点，进行跳点的存储
        Point point_temp(new_x, new_y, dx, dy);
        jumpPoints.emplace_back(point_temp);
        
    }
}




/**
 * @brief   Recursively search jumpPoint
 *          递归地查找跳点
 *          该递归函数会在---一个方向上---一直搜索，直到碰到障碍物、地图边界、跳点
 *          方向包括直线搜索方向和对角搜索方向，其中对角搜索方向是先进行直线搜索，
 *          无跳点再进行对角
 * 
 * @param   x           当前点坐标
 * @param   y           当前点坐标
 * @param   dx          扩展方向
 * @param   dy          扩展方向
 * @param   new_x       找到的新跳点坐标
 * @param   new_y       找到的新跳点坐标
 * @return  bool 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-18
 */
bool JPS::jump(int x, int y, int dx, int dy, int& new_x, int& new_y)
{
    new_x = x + dx;
    new_y = y + dy;
    // cout <<"jump call----";
    if (!isFree(new_x, new_y))
        return false;
    
    if (new_x == goal_point(0) && new_y == goal_point(1))
        return true;
    
    if (hasForced(new_x, new_y, dx, dy))
        return true;
    

    // 继续在当前方向上进行深度搜索，直到return
    // norm1 == 0 不可能，只会在起始点处才为0
    // norm1 == 1 即直线搜索，直接return jump()进行递归搜索，
    // 因为搜索方向只有一个，一直沿这个方向进行递归搜索即可
    // norm1 == 2 即对角搜索，需要先沿两个直线进行搜索，然后再进行对角搜索
    // 反应在程序中，即先执行for循环中的jump进行直线搜索，然后再执行return jump
    const int id = (dx+1) + 3*(dy+1);
    const int norm1 = abs(dx) + abs(dy);
    int num_neib = jps2D->nsz[norm1][0];
    // 此处减一目的是：沿两个直线方向进行递归迭代搜索 第三次迭代正好是对角递归迭代
    // 需要先进行完直线递归迭代搜索后，再进行对角移动一格递归迭代搜索
    for (int k(0); k < num_neib-1; k++)
    {
        int new_new_x, new_new_y;
        if (jump(new_x, new_y, jps2D->ns[id][0][k], jps2D->ns[id][1][k], 
        new_new_x, new_new_y))// 后面不可加分号，错加分号导致了程序的结果不符合预期
            return true;
    }

    return jump(new_x, new_y, dx, dy, new_x, new_y);
}


/**
 * @brief   检查当前节点是否有强迫邻居，todolist这块我感觉作者开源的代码中有点问题！！！！
 * 这部分困扰了我很久，一直百思不得其解！！！！！todolist
 * @param   x           被检查的点
 * @param   y           被检查的点
 * @param   dx          扩展方向
 * @param   dy          扩展方向
 * @return  bool 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-18
 */
inline bool JPS::hasForced(int x, int y, int dx, int dy)
{
    // 扩展方向dx dy和id是以一一对应的
    const int id = dx+1 + 3*(dy+1);
    
    for (int fn = 0; fn < 2; fn++)
    {
        int nx = x + jps2D->f1[id][0][fn];
        int ny = y + jps2D->f1[id][1][fn];
        int tx = x + jps2D->f2[id][0][fn];
        int ty = y + jps2D->f2[id][1][fn];

        // 这部分感觉条件不是很充分，需要修正一下todolist 还需要验证一些墙皮节点处无障碍物！！！
        if (isOccupied(nx, ny) && isFree(tx,ty))
        {
            cout <<"has forced"<<endl;
            return true;
        }
    }
    return false;
}










constexpr int JPS2DNeighb::nsz[3][2];

/**
 * @brief   测试ns f1 f2是否正确
 * 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-22
 */
void JPS2DNeighb::print() 
{
  for(int dx = -1; dx <= 1; dx++) 
  {
    for(int dy = -1; dy <= 1; dy++) 
    {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
      {
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
        printf("                f2: [%d, %d]\n", f2[id][0][i], f2[id][1][i]);

      }
    }
  }
}

/**
 * @brief   完成ns f1 f2 初始化
 * 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-18
 */
JPS2DNeighb::JPS2DNeighb()
{
    int id = 0;
    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int norm1 = std::abs(dx) + std::abs(dy);
            for (int dev = 0; dev < nsz[norm1][0]; dev++)
            {
                Neighb(dx, dy, norm1, dev, ns[id][0][dev], ns[id][1][dev]);
            }

            for (int dev = 0; dev < nsz[norm1][1]; dev++)
            {
                FNeighb(dx, dy, norm1, dev, f1[id][0][dev], f1[id][1][dev]
                ,f2[id][0][dev], f2[id][1][dev]);
            }
            id ++;
        }
    }
}

void JPS2DNeighb::Neighb(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
    // 8 1 3
    switch (norm1)
    {
        case 0://startpoint
            switch (dev)
            {
                case 0: tx=1; ty=0; return;
                case 1: tx=-1; ty=0; return;
                case 2: tx=0; ty=1; return;
                case 3: tx=1; ty=1; return;
                case 4: tx=-1; ty=1; return;
                case 5: tx=0; ty=-1; return;
                case 6: tx=1; ty=-1; return;
                case 7: tx=-1; ty=-1; return;
            }

        case 1://straight recursion
            tx = dx; ty = dy; return;

        case 2://diagonal recursion
            switch(dev)
            {
                case 0: tx = dx; ty = 0; return;
                case 1: tx = 0; ty = dy; return;
                case 2: tx = dx; ty = dy; return;
            }
    }
}

/**
 * @brief   描述
 * 
 * @param   dx          参数描述
 * @param   dy          参数描述
 * @param   norm1       一范数，即曼哈顿距离，
 *                      由此可以判定是直线跳跃还是对角线跳跃
 * @param   dev         参数描述
 * @param   fx          参数描述
 * @param   fy          参数描述
 * @param   nx          参数描述
 * @param   ny          参数描述
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-17
 */
void JPS2DNeighb::FNeighb(int dx, int dy, int norm1, int dev,
                 int& fx, int& fy, int& nx, int& ny)
{
    // 0 2 2
    switch (norm1)
    {
        case 1://straight forced neighbor
            switch (dev)
            {
                case 0: fx=0; fy=1; break;
                case 1: fx=0; fy=-1; break;
            }

            // switch order if different direction
            if (dx == 0)
                fx = fy, fy = 0;
            nx = dx + fx;
            ny = dy + fy;
            return;// 两天的bug出在这个地方，case的最后一定要写break或者return，否则会继续执行后面的case里面的语句！！！！
        case 2://diagonal forced neighbor
            switch(dev)
            {
                case 0:
                    fx = -dx; fy = 0;
                    nx = -dx; ny = dy;
                    return;
                case 1:
                    fx = 0; fy = -dy;
                    nx = dx; ny = -dy;
                    return;
            }
    }
}


inline bool JPS::isOccupied(const int& x, const int& y)
{
    return x < map_size(0) && x >= 0 
        && y < map_size(1) && y >= 0
        && map1[x][y] > free_flag;
}

inline bool JPS::isFree(const int& x, const int& y)
{
    return x < map_size(0) && x >= 0 
    && y < map_size(1) && y >= 0
    && map1[x][y] == free_flag;
}







