#pragma once


/* A_star算法 hpp*/

#include <iostream>
#include <vector>
#include <list>
#include <queue>

using namespace std;
// namespace A_Satr
// {

// }

/* 常量定义 */
const float kCost_straight = 10;
const float kconst_diagonal = 14;
#define INFINITY 100000000
// 节点状态
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

struct Point
{
    float x;
    float y;
    float F, G, H;
    char point_State{};
    Point *parent;
    //构造函数  参数初始化列表---
    Point(float _x, float _y) : x(_x), y(_y), F(0), G(INFINITY), H(0), point_State(NOT_EXPAND),\
    parent(NULL)
    {}
};

typedef Point* PointPtr;

// 优先级队列---priority_queue中，元素之间的比较（通常使用 < 运算符或提供的自定义比较器）用于确定元素的优先级关系,也就是优先级的判定标准。
// 在c++中，class和struct的唯一区别就是默认的访问权限不同，
// 成员的默认权限，class默认为私有，struct默认为公有,所以下面这个地方用struct或者class都可以,看个人习惯
/* ---第二种方法---自定义优先级队列的比较器,要使用重载函数---调用运算符 operator() 来实现自定义比较器*/
// 一般定义数据类型，使用struct；实现某项功能使用class
struct NodeComparator
{
    bool operator() (PointPtr node1, PointPtr node2)
    {
        // 第二个对象的元素 小于 第一个对象的元素时，才真，说明较小的值具有较高的优先级
        return node1->F > node2->F;
    }
};


class Astar
{
    public:
        void InitAstar(const vector<vector<int>> &_map);
        //isIgnoreCorner 是否可以绊住情况下对角线通过
        list<Point *> GetPath(Point& start_Point, Point& end_Point,bool isIgnoreCorner);

    private:


        //判断点是否在openlist中
        Point* isInlist(const Point *curPoint) const;
        //得到G
        float getG(Point *parentPoint, Point *curPoint);
        //得到H
        float getH(Point *curPoint,Point *endPoint);
        //得到F
        float getF(Point *curPoint);

        //得到周边的节点
        vector<Point*> getSurroundPoint(const Point* curPoint,bool isIgnoreCorner ) const;

        //搜索路径 因为searchPath函数需要修改openlist和closelist 所以不能是常量成员函数 也就是不能在最后+ const
        Point* searchPath(Point& startPoint, Point& endPoint,bool isIgnoreCorner);
        //判断是否可以到达
        bool isCanreach(const Point* curPoint,const Point* targetPoint, bool isIgnoreCorner) const;


    private:
        //地图 使用二维数组来表示
        vector< vector<int> > map1;
        // 优先级队列 F值也就是代价越小优先级越高！
        std::priority_queue<Point*, vector<Point*>, NodeComparator> openList;
        // list<Point *> openList;
        // list<Point *> closeList;
};