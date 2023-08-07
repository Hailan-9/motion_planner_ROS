#include <iostream>
#include <math.h>
#include "astar_gridmap_searcher/astar.hpp"


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
    extraG = (extraG == 1) ? kCost_straight : kconst_diagonal;
    // parentG = curPoint->parent == NULL ? 0 : curPoint->parent->G;
    parentG = parentPoint->G;
    return extraG + parentG;
}




//得到H
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


Point* Astar::searchPath(Point& startPoint, Point& endPoint,bool isIgnoreCorner)
{
    //首先是起始点添加进openlist
    //openList.push_back(&startPoint);
    //置入起点，拷贝开辟一个节点，内外隔离
    Point* temp_startPoint = new Point(startPoint.x,startPoint.y);
    temp_startPoint->G = 0;
    temp_startPoint->point_State = IN_OPEN_SET;

    openList.push(temp_startPoint);
    Point* curPoint;
    do{

        //  寻找开启列表中F值最低的点，称其为当前点
        //  curPoint = getLeastFpoint();
        curPoint = openList.top();
        openList.pop();
        curPoint->point_State == IN_CLOSE_SET;
        // openList.remove(curPoint);
        // closeList.push_back(curPoint);


        auto surroundPoint = getSurroundPoint(curPoint,isIgnoreCorner);

        // int count = 0;
        // for (auto it = surroundPoint.begin(); it != surroundPoint.end(); ++it) {
        //     count++;
        // }
        // std::cout << "List capacity: " << count << std::endl;
        // cout<<"test4"<<endl;

        for(auto p :  surroundPoint)
        {
            // 不在openlist中
            if(p->G == INFINITY && p->point_State == NOT_EXPAND)
            {
                p->parent = curPoint;
                p->G = getG(curPoint,p);
                p->H = getH(p,&endPoint);
                p->F = getF(p);
                p->point_State == IN_OPEN_SET;
                openList.push(p);
            }
            else
            {
                if(getG(curPoint,p) < p->G)
                {
                    p->parent = curPoint;
                    p->G = getG(curPoint,p);
                    // p->H = getH(p,endPoint);
                    p->F = getF(p);
                    cout<<"test"<<endl;
                }   

            }

        }

    }while(endPoint.G != INFINITY);

    if(openList.empty())
    {
        return NULL;
    }
    else
    {
        endPoint.parent = curPoint;
    }
    return &endPoint;

}


list<Point*> Astar::GetPath(Point& start_Point, Point& end_Point,bool isIgnoreCorner)
{

    Point* result = searchPath(start_Point, end_Point,isIgnoreCorner);

    if(result)
    {
        cout<<"result: "<<result->x<<" "<<result->y<<endl;
    }
    else
    {
        cout <<"空指针"<<endl;
    }

    list<Point*> path;
    while(result)
    {
        path.push_front(result);
        result = result->parent;
    }
    //清空列表
    std::priority_queue<Point*, vector<Point*>, NodeComparator> empty_queue;
    openList.swap(empty_queue);

    return path;
}


//判断是否可以到达
bool Astar::isCanreach(const Point* curPoint, const Point* targetPoint,bool isIgnoreCorner) const
{


    if(targetPoint->x <0 || targetPoint->x > map1.size() - 1 ||
       targetPoint->y <0 || targetPoint->y > map1[0].size() -1 ||
       curPoint->x == targetPoint->x&&curPoint->y == targetPoint->y
       ||map1[targetPoint->x][targetPoint->y] ==100
       || targetPoint->point_State == IN_CLOSE_SET )
    {
        return false;
    }
    else
    {
        if(fabs(curPoint->x - targetPoint->x) + fabs(curPoint->y - targetPoint->y) ==1 )
        {
            return true;
        }
        else
        {
            //判断斜对角是否被绊住 应该有四个斜对角 四种case
            if(map1[curPoint->x][curPoint->y+1] ==100&&\
            ( (curPoint->x+1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x-1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x][curPoint->y-1] ==100&&\
            ( (curPoint->x-1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ||(curPoint->x+1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x-1][curPoint->y] ==100&&\
            ( (curPoint->x-1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x-1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x+1][curPoint->y] ==100&&\
            ( (curPoint->x+1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x+1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else
            {
                return true;
            }

        }
    }
}
vector<Point*> Astar::getSurroundPoint(const Point* curPoint,bool isIgnoreCorner ) const
{

    vector<Point*> surroundpoints;

    for(int x = curPoint->x - 1; x<=curPoint->x + 1;x++)
    {
        for(int y = curPoint->y - 1; y<=curPoint->y + 1;y++)
        {
            if( isCanreach(curPoint,new Point(x,y), isIgnoreCorner) )
            {
                surroundpoints.emplace_back(new Point(x,y));
            }
        }
    }

    return surroundpoints;
}