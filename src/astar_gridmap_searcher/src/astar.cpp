#include <iostream>
#include <fstream>
#include <math.h>
#include "astar_gridmap_searcher/astar.hpp"
extern std::ofstream outfile;
using namespace A_Star;

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
}

void Astar::reset()
{
    expanded_nodes.clear();
    path_nodes.clear();
    std::priority_queue<Point*, vector<Point*>, NodeComparator> empty_queue;
    openList.swap(empty_queue);
    // 被探索过的每个节点地址都是固定的 牛
    cout <<"reset----"<<endl;
    cout <<"use_node_num:"<<use_node_num<<endl;
    cout <<"iter_num:"<<iter_num<<endl;

    cout <<"capacity:"<<path_node_pool.capacity()<<endl;
    cout <<"size:"<<path_node_pool.size()<<endl;


    for (int i=0; i < use_node_num; i++)
    {
        PointPtr node = path_node_pool[i];
        node->parent = NULL;
        node->point_State = NOT_EXPAND;
    }

    use_node_num = 0;
    iter_num = 0;
}

// todolist这个地方有点问题 参数没有传入
void Astar::setParam(const ros::NodeHandle& nh)
{
    nh.param("astar_node/allocate_num", allocate_num, 10000);
    cout <<"param set finish-------------:"<<allocate_num<<endl;
}


void Astar::InitAstar(const vector<vector<int>> &_map)
{
    map1 = _map;
}

// todolist---有关G的计算，之前算的有的问题，但是改过来了！
//得到G
float Astar::getG(Point *parentPoint, Point *curPoint)
{
    float extraG;
    float parentG;
    extraG = fabs(curPoint->x - parentPoint->x) + fabs(curPoint->y - parentPoint->y);
    extraG = (extraG == 1) ? kCost_straight : kCost_diagonal;
    // parentG = curPoint->parent == NULL ? 0 : curPoint->parent->G;
    parentG = parentPoint->G;
    return extraG + parentG;
}

//得到H todolist需要改写 有几种的启发式函数 都需要写上，并根据传参的不同选择不同的启发式函数
float Astar::getH(Point *curPoint,Point *endPoint)
{
    float costH;
    costH = pow( (curPoint->x - endPoint->x),2) + pow( (curPoint->y - endPoint->y),2);
    return sqrt(costH);
}

//得到F
float Astar::getF(Point *curPoint)
{
    return curPoint->G + curPoint->H;
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


int Astar::searchPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
    // 首先是起始点添加进openlist
    Point* curPoint = path_node_pool[0];
    curPoint->x = startPoint.x;
    curPoint->y = startPoint.y;
    curPoint->pos = startPoint.pos;
    curPoint->G = 0;
    curPoint->H = getH(curPoint,&endPoint);
    curPoint->F = getF(curPoint);
    curPoint->parent = NULL;
    curPoint->point_State = IN_OPEN_SET;

    openList.push(curPoint);
    use_node_num += 1;
    expanded_nodes.insert(curPoint->pos,curPoint);

    do{
        // 寻找开启列表中F值最低的点，称其为当前点
        curPoint = openList.top();
        /* -----------------Determine if the destination has been reached--------------------- */
        bool reach_goal = abs(curPoint->x - endPoint.x) <=1 && abs(curPoint->y - endPoint.y) <=1;
        if(reach_goal)
        {
            retrievePath(curPoint);
            cout <<"path search success!!!!!!"<<endl;
            cout << "use node num: " << use_node_num << endl;
            cout << "iter num: " << iter_num << endl;
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

                        openList.push(neighbor_node);
                        expanded_nodes.insert(neighbor_node->pos, neighbor_node);

                        use_node_num += 1;

                        if (use_node_num == allocate_num)
                        {
                            cout <<"***run out of node_pool memory***" <<endl;
                            cout <<"use_node_num: "<<use_node_num<<endl;
                            cout <<"iterm_num: "<<iter_num<<endl;

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
    return NO_PATH;
}


vector<Eigen::Vector2f> Astar::GetPath()
{

    vector<Eigen::Vector2f> path;

    for (int i(0); i < path_nodes.size(); i++)
    {
        path.push_back(path_nodes[i]->pos);
        outfile <<"(" << path_nodes[i]->pos(0)<<","<<path_nodes[i]->pos(1)<<")"<<endl;
    }
    outfile <<"*****************************************************"<<endl;
    return path;
}


//判断是否可以到达
bool Astar::isCanreach(const Point* curPoint, const Eigen::Vector2f& targetPoint,bool isIgnoreCorner) const
{

    // if(targetPoint(0) <0 || targetPoint(0) > map1.size() - 1 ||
    //    targetPoint(1) <0 || targetPoint(1) > map1[0].size() -1 ||
    if(targetPoint(0) <0 || targetPoint(0) > map1[0].size() - 1 ||
    targetPoint(1) <0 || targetPoint(1) > map1.size() -1 ||
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
