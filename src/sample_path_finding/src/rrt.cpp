/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    rrt.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-26
 */

/*
采样是基于栅格地图进行的，采样点是世界坐标系中的点，不是栅格坐标系中的点。

算法主要流程和构成：
    step1:采样函数-sample
    step2:采样点的最近邻搜索-near，优化的一个方法是使用kd-tree
    step3:产生new节点的-steer函数
    step4:产生边的-edge函数
    step5:碰撞检测函数

*/

#include "sample_path_finding/rrt.hpp"



extern std::ofstream outfile_rrt;
extern std::ofstream outfile_rrt1;

// using namespace name;



/**
 * @brief   描述
 * 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-27
 */
void RRT::init(const vector<vector<int>> &_map, Eigen::Vector2f origin, ros::NodeHandle& nh)
{

    // 载入地图
    map = _map;
    map_size <<map.size(), map[0].size();
    map_origin = origin;


    nh.param("allocate_num", allocate_num, 10000);
    path_node_pool.resize(allocate_num);
    // 开辟内存
    for (int i(0); i < allocate_num; i++)
    {
        path_node_pool[i] = new Point;
    }

    use_node_num = 0;
    


    ROS_WARN("algorithm init over!!!!!!!!!!!!!!!!!");

}




void RRT::setParam(ros::NodeHandle& nh)
{
    nh.param("allocate_num", allocate_num, 10000);
    nh.param("resolution", resolution, 0.05f);
    nh.param("collision_check_num", collision_check_num, 4.0f);
    nh.param("stepSize", stepSize, 0.2f);
    nh.param("goal_tolerance", goal_tolerance, 0.2f);
    nh.param("max_iter_num", max_iter_num, 5000);
    cout <<"whether has goal_tolerance: " <<nh.hasParam("goal_tolerance")<<endl;
    // nh.param("min_val_x", min_val(0), 0.0);
    // nh.param("min_val_y", min_val(1), 0.0);
    // nh.param("max_val_x", max_val(0), 0.0);
    // nh.param("max_val_y", max_val(1), 0.0);
    // 随机撒点采样的最大最小范围
    min_val = map_origin;
    max_val(0) = map_size(0) * resolution + map_origin(0);
    max_val(1) = map_size(1) * resolution + map_origin(1);
    tree_pub = nh.advertise<visualization_msgs::Marker>("sample_tree",1);
    xnew_Pub = nh.advertise<visualization_msgs::Marker>("sample_xnew",1);

    cout <<"rrt param set finish-------------" << endl;
    cout <<"min_val-------------" << min_val(0)<<" "<<min_val(1)<<endl;
    cout <<"max_val-------------" << max_val(0)<<" "<<max_val(1)<<endl;



    // 生成的树进行可视化----增量式可视化处理
    tree.header.frame_id = "map";
    tree.header.stamp = ros::Time::now();
    // LINE_LIST 用于显示连接的线段
    tree.type = visualization_msgs::Marker::LINE_LIST;
    tree.action = visualization_msgs::Marker::ADD;
    // tree.markers[0].ns = "rrt_tree";
    tree.id = 0;

    tree.pose.orientation.x = 0.0;
    tree.pose.orientation.y = 0.0;
    tree.pose.orientation.z = 0.0;
    tree.pose.orientation.w = 1.0;

    tree.color.a = 1.0;
    tree.color.r = 0.0;
    tree.color.g = 1.0;
    tree.color.b = 0.0;

    tree.scale.x = resolution * 0.1; // line width

    // 发布新生成的树，需要起点和终点，即x_nearest和x_new
    // tree.points.emplace_back(sub_tree_start);
    // tree.points.emplace_back(sub_tree_end);




    // 新产生的xnew采样点的可视化
    // 蓝色
    xnew_visual.header.frame_id = "map";
    xnew_visual.header.stamp = ros::Time::now();
    xnew_visual.type = visualization_msgs::Marker::CUBE_LIST;
    xnew_visual.action = visualization_msgs::Marker::ADD;
    xnew_visual.id = 3;

    xnew_visual.pose.orientation.x = 0.0;
    xnew_visual.pose.orientation.y = 0.0;
    xnew_visual.pose.orientation.z = 0.0;
    xnew_visual.pose.orientation.w = 1.0;

    xnew_visual.color.a = 1.0;
    xnew_visual.color.r = 0.0;
    xnew_visual.color.g = 0.0;
    xnew_visual.color.b = 1.0;

    xnew_visual.scale.x = resolution * 0.5;
    xnew_visual.scale.y = resolution * 0.5;
    xnew_visual.scale.z = resolution * 0.001;


}



RRT::~RRT()
{
    for (int i(0); i < allocate_num; i++)
    {
        delete path_node_pool[i];
    }
}


/**
 * @brief   在栅格地图中进行随机撒点采样 C-Space配置空间，---随机数满足均匀分布---
 *          随机采样是一项重要的步骤，它用于在---配置空间---中随机生成新的节点，
 *          以便逐步构建运动规划的路径。
 * 
 * @param   min_val     采样点的xy二维范围
 * @param   max_val     采样点的xy二维范围
 * @return  Point 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-27
 */
Point RRT::randomSample()
{
    // 用于生成真正的随机数种子的一个类
    // 使用该类获得随机数生成器的种子，以确保随机数具有一定的随机性
    std::random_device rd;
    // 伪随机数生成器引擎，能够产生高质量、良好随机性的伪随机数序列
    std::mt19937 gen(rd());
    // 用于生成均匀分布的随机实数-------------------
    std::uniform_real_distribution<float> distx(min_val(0),max_val(0));
    std::uniform_real_distribution<float> disty(min_val(1),max_val(1));
    // 生成随机数是通过分布类来实现的，比如上一行的这个类，它需要一个随机数引擎对象来驱动随机数的生成
    Point random_point;
    Eigen::Vector2f pos(distx(gen),disty(gen));
    random_point.set_w_pos(pos);
    return random_point;
}


/**
 * @brief   todolist描述 先使用原始的查找 后续再实现kdtree
 * 
 * @param   x_rand      参数描述
 * @return  Point 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-28
 */
PointPtr RRT::getNearestNode(Point x_rand)
{
    int min_index = -1;
    float min_distance = std::numeric_limits<float>::max();

    for (int i = 0; i < node_list.size(); i++)
    {
        float square_distance = 
            std::pow(node_list[i]->w_x - x_rand.w_x, 2) +
            std::pow(node_list[i]->w_y - x_rand.w_y, 2);
        if (square_distance < min_distance)
        {
            min_distance = square_distance;
            min_index = i;
        }
    }

    return node_list[min_index];
}


/**
 * @brief   x_rand只是作为树生长的方向
 * 
 * @param   x_rand      参数描述
 * @return  Point 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-28
 */
Point RRT::steer(Point x_rand, PointPtr x_nearest)
{
    float theta = atan2(x_rand.w_y - x_nearest->w_y, x_rand.w_x - x_nearest->w_x);

    Point x_new;

    x_new.w_x = x_nearest->w_x + cos(theta)*stepSize;
    x_new.w_y = x_nearest->w_y + sin(theta)*stepSize;

    outfile_rrt << "theta: " << theta << " "
    << "x_new: " << x_new.w_x <<" " << x_new.w_y <<endl;

    return x_new;
}


/**
 * @brief   对x_new节点和其与最近节点连线即树枝进行碰撞检测
 * 
 * @param   x_new       参数描述
 * @param   x_nearest   参数描述
 * @return  bool 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-28
 */
bool RRT::collisionFree(Point x_new, PointPtr x_nearest)
{
    float dx_check = (x_new.w_x - x_nearest->w_x) / collision_check_num;
    float dy_check = (x_new.w_y - x_nearest->w_y) / collision_check_num;
    float nearest_x = x_nearest->w_x;
    float nearest_y = x_nearest->w_y;

    if(map[world2Gridmap(x_new.w_x, x_new.w_y)(0)]
    [world2Gridmap(x_new.w_x, x_new.w_y)(1)] == 100)
    {
        return false;
    }
    // 分几次进行进行采样递进检测
    else
    {
        for (int i = 0; i < collision_check_num; i++)
        {
            nearest_x += dx_check;
            nearest_y += dy_check;

            if(map[world2Gridmap(nearest_x, nearest_y)(0)]
            [world2Gridmap(nearest_x, nearest_y)(1)] == 100)
            {
                return false;
            }
        }
    }
    return true;
}


/**
 * @brief   描述
 * 
 * @param   startPoint  参数描述
 * @param   endPoint    参数描述
 * @return  int 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-28
 */
int RRT::pathFind(const Point& startPoint, const Point& endPoint)
{
    Point x_rand;
    PointPtr x_nearest;
    Point x_new;
    PointPtr x_new_ptr;

    PointPtr startPointPtr = path_node_pool[0];
    startPointPtr->set_w_pos(startPoint.w_x, startPoint.w_y);


    Eigen::Vector2d dist_goal;
    // 新生成的子树
    geometry_msgs::Point sub_tree_start, sub_tree_end;
    geometry_msgs::Point xnew_visual_point;

    node_list.emplace_back(startPointPtr);

    use_node_num += 1;




    ROS_WARN("start sample path finding");
    /* ---------------sanple finding--------------- */
    // while (1)
    while (iterm_num < max_iter_num)
    {
        iterm_num++;
        ros::Duration(1).sleep();

        // cout << "use_node_num: "<< use_node_num <<endl;
        x_rand = randomSample();
        x_nearest = getNearestNode(x_rand);
        x_new = steer(x_rand, x_nearest);

        bool collision_free_flag = collisionFree(x_new, x_nearest);
        if (!collision_free_flag)
        {
            continue;
        }
        else
        {
            // x_new新采样点的可视化显示
            xnew_visual_point.x = x_new.w_x;
            xnew_visual_point.y = x_new.w_y;

            xnew_visual.points.emplace_back(xnew_visual_point);
            xnew_Pub.publish(xnew_visual);


            sub_tree_start.x = x_nearest->w_x;
            sub_tree_start.y = x_nearest->w_y;
            sub_tree_end.x = x_new.w_x;
            sub_tree_end.y = x_new.w_y;

            tree.points.emplace_back(sub_tree_start);
            tree.points.emplace_back(sub_tree_end);
            // 可视化发布
            tree_pub.publish(tree);
    


            x_new_ptr = path_node_pool[use_node_num];
            x_new_ptr->parent = x_nearest;
            x_new_ptr->set_w_pos(x_new.w_x, x_new.w_y);

            node_list.emplace_back(x_new_ptr);

            use_node_num += 1;

            // 在目标点附近一定范围内，就认为搜索结束
            dist_goal<<x_new.w_x - endPoint.w_x, x_new.w_y - endPoint.w_y;
            if (dist_goal.squaredNorm() < goal_tolerance)
            {
                outfile_rrt1 <<"iter_num: "<<iterm_num<<endl;
                outfile_rrt1 <<"goal_tolerance: "<<goal_tolerance<<endl;
                outfile_rrt1 <<"dist_goal.squaredNorm(): " <<dist_goal.squaredNorm()<<endl;
                outfile_rrt1 <<"x_new_ptr: " <<x_new_ptr->w_x << " " << x_new_ptr->w_y <<endl;
                retrievePath(x_new_ptr);
                return REACH_END;
            }

            if (use_node_num == allocate_num)
            {
                cout <<"***run out of node_pool memory***" <<endl;
                cout <<"use_node_num: "<<use_node_num<<endl;
                
                // outfile <<"*****************************************************"<<endl;
                // outfile <<"***run out of node_pool memory***" <<endl;
                // outfile <<"total cost(G): "<<curPoint->G<<endl;
                // outfile <<"use_node_num: "<<use_node_num<<endl;

                return NO_PATH;
            }

        }



    }

    cout <<"use_node_num: "<<use_node_num<<endl;
    ROS_WARN(" over iterm_num!");
    return NO_PATH;
}


/**
 * @brief   描述
 * 
 * @param   end_node    参数描述
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-08-28
 */
void RRT::retrievePath(PointPtr end_node)
{
    // 这俩是不同的指针了 分离
    // end_node这个指针变量不会跟着cur_node的变化而变化
    // cur_node和end_node是两个独立的指针变量
    // 后面语句中，指针变量cur_node不断地指向新的内存地址
    PointPtr cur_node = end_node;
    path_nodes.push_back(cur_node);
    
    outfile_rrt1 <<"retrievePath"<<endl;
    while (cur_node->parent != NULL)
    {
        cur_node = cur_node->parent;
        path_nodes.push_back(cur_node);
        outfile_rrt1 << cur_node->w_x <<" "<<cur_node->w_y<<endl;
    }

    outfile_rrt1 <<"retrieve over"<<endl;
    reverse(path_nodes.begin(), path_nodes.end());
}


vector<Eigen::Vector2f> RRT::getPath()
{
    Eigen::Vector2f offset_pos;
    offset_pos << resolution * 0.5, resolution * 0.5;
    vector<Eigen::Vector2f> path;

    for (int i(0); i < path_nodes.size(); i++)
    {
        path.push_back(path_nodes[i]->w_pos);
        // outfile <<"(" << path_nodes[i]->w_pos(0)<<","<<path_nodes[i]->w_pos(1)<<")"<<endl;
    }
    return path;
}




void RRT::reset()
{
    // todolist可视化数据清空

    for (int i=0; i < use_node_num; i++)
    {
        PointPtr node = path_node_pool[i];
        node->parent = NULL;
    }
    tree.points.clear();
    xnew_visual.points.clear();

    path_nodes.clear();
    node_list.clear();

    use_node_num = 0;
    iterm_num = 0;
}



// 世界坐标系（也就是地图坐标系）-->栅格地图坐标系
Eigen::Vector2f RRT::world2Gridmap(float w_x, float w_y)
{
    // ROS_INFO("function---world2Gridmap----");
    Eigen::Vector2f result;
    if (w_x < map_origin(0) || w_y < map_origin(1))
    {
        result << -1,-1;
        return result;
    }

    int temp_x = int((w_x - map_origin(0)) / resolution);
    int temp_y = int((w_y - map_origin(1)) / resolution);

    if (temp_x < map_size(0) && temp_y < map_size(1))
    {
        result << temp_x,temp_y;
        return result;
    }
}

// 栅格地图坐标系-->世界坐标系（也就是地图坐标系）
Eigen::Vector2f RRT::gridmap2World(float gm_x, float gm_y)
{
    // ROS_INFO("function---gridmap2World----");
    Eigen::Vector2f result;
    if (gm_x > map_size(0) || gm_y > map_size(1))
    {
        result << -1, -1;
        return result;
    }

    float temp_x = gm_x * resolution + map_origin(0);
    float temp_y = gm_y * resolution + map_origin(1);

    if (temp_x > map_origin(0) && temp_y > map_origin(1))
    {
        result << temp_x, temp_y;
        return result;
    }
}

