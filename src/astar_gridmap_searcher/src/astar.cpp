#include <iostream>
#include <fstream>
#include <math.h>
#include "astar_gridmap_searcher/astar.hpp"
extern std::ofstream outfile;
extern std::ofstream outfile1;

using namespace Grid_Path_Search;

Astar::~Astar()
{
    for (int i(0); i < allocate_num; i++)
    {
        delete path_node_pool[i];
    }
    cout <<"执行astar析构函数---------"<<endl;
}

void Astar::init()
{
    path_node_pool.resize(allocate_num);
    cout <<"-------2.1"<<endl;

    for (int i(0); i < allocate_num; i++)
    {
        path_node_pool[i] = new Point;
    }
    use_node_num = 0;
    iter_num = 0;


    // closeset中的节点可视化，即被访问过的节点的可视化
    // 蓝色
    node_closed_visual.header.frame_id = "map";
    node_closed_visual.header.stamp = ros::Time::now();
    node_closed_visual.type = visualization_msgs::Marker::CUBE_LIST;
    node_closed_visual.action = visualization_msgs::Marker::ADD;
    node_closed_visual.id = 3;

    node_closed_visual.pose.orientation.x = 0.0;
    node_closed_visual.pose.orientation.y = 0.0;
    node_closed_visual.pose.orientation.z = 0.0;
    node_closed_visual.pose.orientation.w = 1.0;

    node_closed_visual.color.a = 1.0;
    node_closed_visual.color.r = 0.0;
    node_closed_visual.color.g = 1.0;
    node_closed_visual.color.b = 0.0;

    node_closed_visual.scale.x = resolution * 1;
    node_closed_visual.scale.y = resolution * 1;
    node_closed_visual.scale.z = resolution * 0.001;


}

void Astar::reset()
{
    cout <<"reset----"<<endl;
    expanded_nodes.clear();
    path_nodes.clear();

    // node_visited_visual.points.clear();
    node_closed_visual.points.clear();

    std::priority_queue<Point*, vector<Point*>, NodeComparator> empty_queue;
    openList.swap(empty_queue);
    // 被探索过的每个节点地址都是固定的 牛
    // cout <<"capacity:"<<path_node_pool.capacity()<<endl;
    // cout <<"size:"<<path_node_pool.size()<<endl;


    for (int i=0; i < use_node_num; i++)
    {
        PointPtr node = path_node_pool[i];
        node->parent = NULL;
        node->point_State = NOT_EXPAND;
    }

    use_node_num = 0;
    iter_num = 0;

}

void Astar::setParam(ros::NodeHandle& nh)
{
    // 获取参数有两种方式：
    // ros::NodeHandle 提供了 param() 函数用于获取参数的值。
    // 当你使用这个函数时，如果提供的参数名以斜杠 / 开头，
    // 那么它会在参数服务器中寻找全局命名空间。如果不以斜杠开头，它将在当前节点的命名空间中查找。

    // 由于我们的参数定义为 /astar_node/allocate_num，你可以使用全局命名空间获取它的值，
    // 也可以使用相对于当前节点的命名空间。
    // 使用全局命名空间获取："/astar_node/allocate_num"
    // 使用相对命名空间获取："allocate_num"
    // 无论你使用哪种方式，都可以获取到参数的值。

    // nh.param("/astar_node/allocate_num", allocate_num, 10000);
    nh.param("allocate_num", allocate_num, 10000);
    nh.param("resolution", resolution, 0.05);
    nh.param("heuristic_options", Heuristic_Options, 2);
    nh.param("weight_g", weight_g, 1.0);
    nh.param("weight_h", weight_h, 1.0);

    tie_breaker = 1.0 + 1.0 / 1000;

    // Heuristic_Options = EUCLIDEAN_DISTANCE;
    // Heuristic_Options = MANHATTAN_DISTANCE;

    
    node_visited_Pub = nh.advertise<visualization_msgs::Marker>("node_visited",1);
    node_closed_Pub = nh.advertise<visualization_msgs::Marker>("node_closed",1);

    cout <<"param set finish-------------:"<<allocate_num<<endl;
}


void Astar::InitAstar(const vector<vector<int>> &_map, Eigen::Vector2f origin)
{
    map1 = _map;
    map_size << map1[0].size(), map1.size();
    map_origin = origin;
}

// todolist---有关G的计算，之前算的有的问题，但是改过来了！
//得到G
float Astar::getG(Point *parentPoint, Point *curPoint)
{
    Eigen::Vector2f temp_vec = parentPoint->pos - curPoint->pos;
    float extraG;
    float parentG;
    extraG = temp_vec.squaredNorm();
    parentG = parentPoint->G;
    return extraG + parentG;
}

//得到H todolist需要改写 有几种的启发式函数 都需要写上，并根据传参的不同选择不同的启发式函数
float Astar::getH(Point *curPoint,Point *endPoint)
{
    float costH;
    Eigen::Vector2f temp_vec = endPoint->pos - curPoint->pos;
    switch (Heuristic_Options)
    {
        case EUCLIDEAN_DISTANCE:
            // 欧几里得
            costH = temp_vec.norm();
            // 平方欧几里德方法
            return tie_breaker * costH;

        case MANHATTAN_DISTANCE:
            return tie_breaker * (fabs(temp_vec(0)) + fabs(temp_vec(1)));

        case DIAGONAL_DISTANCE:
            costH = fabs(temp_vec(0)) + fabs(temp_vec(1)) + (1.414 - 2) * std::min(fabs(temp_vec(0)),fabs(temp_vec(1)));
            return tie_breaker * costH;
    }
}

//得到F
float Astar::getF(Point *curPoint)
{
    return weight_g * curPoint->G + weight_h * curPoint->H;
}

void Astar::retrievePath(PointPtr end_node)
{
    PointPtr cur_node = end_node;
    path_nodes.push_back(cur_node);
    while (cur_node->parent != NULL)
    {
        cur_node = cur_node->parent;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(), path_nodes.end());
}




// 世界坐标系（也就是地图坐标系）-->栅格地图坐标系
Eigen::Vector2f Astar::world2Gridmap(float w_x, float w_y)
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
Eigen::Vector2f Astar::gridmap2World(float gm_x, float gm_y)
{
    // ROS_INFO("function---gridmap2World----");
    Eigen::Vector2f result;
    if (gm_x > map_size(0) || gm_y > map_size(1))
    {
        cout <<"-111111111111111111111111"<<endl;
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



int Astar::searchPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{

    // 坐标系转换
    startPoint.set_pos(world2Gridmap(startPoint.w_x,startPoint.w_y)(0),
    world2Gridmap(startPoint.w_x,startPoint.w_y)(1));

    endPoint.set_pos(world2Gridmap(endPoint.w_x,endPoint.w_y)(0),
    world2Gridmap(endPoint.w_x,endPoint.w_y)(1));

    cout <<"gridmap frame---start: "<<startPoint.x<<","<<startPoint.y<<endl;

    cout <<"gridmap frame---goal: "<<endPoint.x<<","<<endPoint.y<<endl;


    geometry_msgs::Point node_closed_point;
    geometry_msgs::Point node_visited_point;


    // 首先是起始点添加进openlist
    Point* curPoint = path_node_pool[0];
    curPoint->x = startPoint.x;
    curPoint->y = startPoint.y;
    curPoint->pos = startPoint.pos;
    curPoint->set_w_pos(startPoint.w_pos);

    curPoint->G = 0;
    curPoint->H = getH(curPoint,&endPoint);
    curPoint->F = getF(curPoint);
    curPoint->parent = NULL;
    curPoint->point_State = IN_OPEN_SET;

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


        Eigen::Vector2f temp_point;
        for(int x = curPoint->x - 1; x<=curPoint->x + 1; x++)
        {
            for(int y = curPoint->y - 1; y<=curPoint->y + 1; y++)
            {
                temp_point <<x, y;
                if( isCanreach(curPoint, temp_point, isIgnoreCorner) )
                {
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
                        neighbor_node->set_pos(x, y);
                        neighbor_node->G = getG(curPoint,neighbor_node);
                        neighbor_node->H = getH(neighbor_node,&endPoint);
                        neighbor_node->F = getF(neighbor_node);
                        neighbor_node->point_State = IN_OPEN_SET;
                        neighbor_node->set_w_pos(gridmap2World(neighbor_node->x, neighbor_node->y));

                        openList.push(neighbor_node);
                        expanded_nodes.insert(neighbor_node->pos, neighbor_node);


                        /* -----------------显示探索的节点------------------*/
                        node_visited_point.x = neighbor_node->w_x + resolution * 0.5;
                        node_visited_point.y = neighbor_node->w_y + resolution * 0.5;




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
                        }
                    }
                }
                else
                {
                    continue;
                }
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
    outfile <<"node_closed_num: "<<node_closed_visual.points.size()<<endl;

    return NO_PATH;
}


vector<Eigen::Vector2f> Astar::GetPath()
{
    Eigen::Vector2f offset_pos;
    offset_pos << resolution * 0.5, resolution * 0.5;
    vector<Eigen::Vector2f> path;

    for (int i(0); i < path_nodes.size(); i++)
    {
        path.push_back(path_nodes[i]->w_pos + offset_pos);
        // outfile <<"(" << path_nodes[i]->w_pos(0)<<","<<path_nodes[i]->w_pos(1)<<")"<<endl;
    }
    return path;
}


//判断是否可以到达
bool Astar::isCanreach(const Point* curPoint, const Eigen::Vector2f& targetPoint,bool isIgnoreCorner) const
{

    if(targetPoint(0) <0 || targetPoint(0) > map1.size() - 1 ||
       targetPoint(1) <0 || targetPoint(1) > map1[0].size() -1 ||
        (curPoint->x == targetPoint(0) && curPoint->y == targetPoint(1))
       || map1[targetPoint(0)][targetPoint(1)] == 100
       )
    {
        return false;
    }
    else
    {
        return true;
        // if(fabs(curPoint->x - targetPoint(0)) + fabs(curPoint->y - targetPoint(1)) == 1)
        // {
        //     return true;
        // }
        // else
        // {
        //     //判断斜对角是否被绊住 应该有四个斜对角 四种case
        //     if(map1[curPoint->x][curPoint->y+1] ==100&&\
        //     ( (curPoint->x+1 == targetPoint(0)&&curPoint->y+1 == targetPoint(1)) 
        //     ||(curPoint->x-1 == targetPoint(0)&&curPoint->y+1 == targetPoint(1)) 
        //     ) )
        //     {
        //         return isIgnoreCorner;
        //     }
        //     else if(map1[curPoint->x][curPoint->y-1] ==100&&\
        //     ( (curPoint->x-1 == targetPoint(0)&&curPoint->y-1 == targetPoint(1)) 
        //     ||(curPoint->x+1 == targetPoint(0)&&curPoint->y-1 == targetPoint(1)) 
        //     ) )
        //     {
        //         return isIgnoreCorner;
        //     }
        //     else if(map1[curPoint->x-1][curPoint->y] ==100&&\
        //     ( (curPoint->x-1 == targetPoint(0)&&curPoint->y+1 == targetPoint(1)) 
        //     ||(curPoint->x-1 == targetPoint(0)&&curPoint->y-1 == targetPoint(1)) 
        //     ) )
        //     {
        //         return isIgnoreCorner;
        //     }
        //     else if(map1[curPoint->x+1][curPoint->y] ==100&&\
        //     ( (curPoint->x+1 == targetPoint(0)&&curPoint->y+1 == targetPoint(1)) 
        //     ||(curPoint->x+1 == targetPoint(0)&&curPoint->y-1 == targetPoint(1)) 
        //     ) )
        //     {
        //         return isIgnoreCorner;
        //     }
        //     else
        //     {
        //         return true;
        //     }

        // }
    }
}
